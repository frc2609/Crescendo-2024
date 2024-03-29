// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Alert;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.BeaverLogger;
// import frc.robot.utils.TunableNumber;

// TODO: add back simulation
// TODO: javadocs
// TODO: override does nothing
// TODO: 'absSetCounter should be removed'

public class ShooterAngle extends SubsystemBase {
  // ** Angle Direction: +ve = Forward, towards elevator. **
  public static final Rotation2d forwardLimit = Rotation2d.fromDegrees(65);
  public static final Rotation2d forwardTolerance = Rotation2d.fromDegrees(3);
  public static final Rotation2d reverseLimit = Rotation2d.fromDegrees(9.1);
  public static final Rotation2d reverseTolerance = Rotation2d.fromDegrees(1);
  public static final Rotation2d setpointTolerance = Rotation2d.fromDegrees(1.5);

  // measure at 90 degrees
  public static final double absoluteEncoderOffset = 0.598 - (90.0 / 360.0);
  // 15:22 chain and 1:81 planetary
  public static final double positionConversionFactor = (15.0 / 22.0) * (1.0 / 81.0) * 360; // deg
  public static final double velocityConversionFactor = positionConversionFactor / 60; // deg/s
  
  // private final TunableNumber anglePGain = new TunableNumber("Shooter Angle P", 0.00003);
  // private final TunableNumber angleDGain = new TunableNumber("Shooter Angle D", 0.001);
  // private final TunableNumber angleFFGain = new TunableNumber("Shooter Angle FF", 0.00004);

  private final CANSparkMax angleMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder relativeEncoder = angleMotor.getEncoder(); // unit: DEGREES
  private final SparkPIDController anglePID = angleMotor.getPIDController();
  private final AbsoluteEncoder absoluteEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private Rotation2d targetAngle = reverseLimit; // used for 'atTarget()' exclusively
  private final Alert absoluteAngleOutOfRange = new Alert("Shooter Absolute Angle Out of Reasonable Range", AlertType.ERROR);
  private final BeaverLogger logger = new BeaverLogger();
  private int absSetCounter = 0;

  /** Creates a new NewShooterAngle. */
  public ShooterAngle() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setInverted(true);
    angleMotor.setSmartCurrentLimit(40);
    // Spark soft limits?

    absoluteEncoder.setZeroOffset(absoluteEncoderOffset);
    absoluteEncoder.setPositionConversionFactor(360);
    relativeEncoder.setPositionConversionFactor(positionConversionFactor);
    relativeEncoder.setVelocityConversionFactor(velocityConversionFactor);

    anglePID.setFeedbackDevice(absoluteEncoder);
    anglePID.setSmartMotionMaxVelocity(10000, 0);
    anglePID.setSmartMotionMinOutputVelocity(0, 0);
    anglePID.setSmartMotionMaxAccel(5000, 0);
    anglePID.setSmartMotionAllowedClosedLoopError(0.5, 0);
    anglePID.setP(0.000055);
    anglePID.setI(0.0); // doesn't do anything
    anglePID.setD(0.001);
    anglePID.setIZone(10.0);
    anglePID.setFF(0.000053);
    
    // anglePID.setP(0.0);
    // anglePID.setI(0.0); // doesn't do anything
    // anglePID.setD(0.002);
    // anglePID.setIZone(0.0);
    // anglePID.setFF(0.0001);
    anglePID.setOutputRange(-1.0, 1.0);

    // armSim.update(0); // setup simulation before periodic() runs for the first time

    SmartDashboard.putBoolean("Overrides/Shooter Angle", false);
    logger.addLoggable("Shooter/Angle/Raw Absolute Position (0-1)", this::getRawAbsolutePosition, true);
    logger.addLoggable("Shooter/Angle/Absolute Position (0-1)", this::getAbsolutePosition, true);
    logger.addLoggable("Shooter/Angle/Absolute Angle (deg)", () -> getAbsoluteAngle().getDegrees(), true);
    logger.addLoggable("Shooter/Angle/Relative Angle (deg)", () -> getRelativeAngle().getDegrees(), true);
    logger.addLoggable("Shooter/Angle/Applied Output (-1-1)", angleMotor::getAppliedOutput, true);
    logger.addLoggable("Shooter/Angle/Accumulated I", anglePID::getIAccum, true);
  }

  @Override
  public void periodic() {
    if (!absoluteAngleOutOfRange.isActive() && relativeEncoder.getVelocity() < 2 && absSetCounter < 5) {
      // reset relative encoder to absolute when not moving and absolute encoder isn't invalid
      relativeEncoder.setPosition(getAbsoluteAngle().getDegrees());
      absSetCounter++;
    }

    // set alerts
    if (pastForwardLimit() || pastReverseLimit()) {
      absoluteAngleOutOfRange.set(true);
      stop(); // in case setAngle/setMotor isn't being called
    } else {
      absoluteAngleOutOfRange.set(false);
    }

    // check tunable numbers
    // if (anglePGain.hasChanged(hashCode()) || angleDGain.hasChanged(hashCode()) || angleFFGain.hasChanged(hashCode())) {
    //   anglePID.setP(anglePGain.get());
    //   anglePID.setD(angleDGain.get());
    //   anglePID.setFF(angleFFGain.get());
    // }

    // set sim mechanism here (using % output from Spark)

    SmartDashboard.putBoolean("Shooter/Angle/At Target", atTarget());
    logger.logAll();
  }

  public void syncRelativeEncoder() {
    relativeEncoder.setPosition(getAbsoluteAngle().getDegrees());
  }

  // set angle using PID
  // will HOLD (read: this function doesn't run run) until 'stop' called
  public void setAngle(Rotation2d angle) {
    if (hasError() && !errorOverridden()) {
      stop();
      return;
    }
    if (angle.getDegrees() == Double.NaN) {
      return; // continue with whatever the motor was doing before
    }
    // clamp angle to allowable setpoints
    SmartDashboard.putNumber("Shooter/Angle/Desired Target (deg)", angle.getDegrees());
    angle = Rotation2d.fromDegrees(MathUtil.clamp(angle.getDegrees(), reverseLimit.getDegrees(), forwardLimit.getDegrees()));
    SmartDashboard.putNumber("Shooter/Angle/Target (deg)", targetAngle.getDegrees());
    // set angle
    targetAngle = angle; // used to check 'atTarget()'
    // adjust PID according to target angle
    if (angle.getDegrees() > 40.0) {
      anglePID.setP(0.000040);
      anglePID.setD(0.003000);
      anglePID.setFF(0.00004);
    } else {
      anglePID.setP(0.000050);
      anglePID.setD(0.002000);
      anglePID.setFF(0.000050);
    }
    anglePID.setReference(angle.getDegrees(), ControlType.kSmartMotion);
  }

  // set motor
  // will obey hasError && limits
  public void setMotor(double percentOutput) {
    SmartDashboard.putNumber("Shooter/Angle/Desired Percent Output", percentOutput);
    if (!errorOverridden()) {
      if (atForwardLimit()) {
        percentOutput = Math.min(percentOutput, 0);
      }
      if (atReverseLimit()) {
        percentOutput = Math.max(percentOutput, 0);
      }
      if (hasError()) {
        percentOutput = 0;
      }
    }
    percentOutput = MathUtil.clamp(percentOutput, -1, 1);
    SmartDashboard.putNumber("Shooter/Angle/Percent Output", percentOutput);
    angleMotor.set(percentOutput);
  }

  public void stop() {
    angleMotor.set(0);
  }

  /**
   * Check whether the absolute angle is within the angle tolerance to the target.
   * @return Whether the absolute angle is within the angle tolerance to the target.
   */
  public boolean atTarget() {
    return Math.abs(targetAngle.getDegrees() - getAbsoluteAngle().getDegrees()) <= setpointTolerance.getDegrees();
  }

  public boolean hasError() {
    return absoluteAngleOutOfRange.isActive(); // || other alerts...
  }

  public boolean errorOverridden() {
    return SmartDashboard.getBoolean("Overrides/Shooter Angle", false);
  }

  // command factories

  public Command getStop() {
    return new InstantCommand(this::stop, this);
  }

  public Command getSetpointControl(Supplier<Double> setpointAxis) {
    return new InstantCommand(() -> setAngle(
      Rotation2d.fromRotations(MathUtil.clamp(setpointAxis.get() * forwardLimit.getDegrees(), reverseLimit.getRotations(), forwardLimit.getRotations()))
    ), this);
  }

  public Command getPercentOutputControl(Supplier<Double> percentOutputAxis) {
    return new RunCommand(() -> setMotor(MathUtil.clamp(percentOutputAxis.get(), -1, 1)), this);
  }

  // get angle (in various formats)

  /**
   * Get the angle of the shooter according to the relative encoder.
   * @return The angle of the shooter as a Rotation2d.
   */
  public Rotation2d getRelativeAngle() {
    return RobotBase.isSimulation() ? targetAngle : Rotation2d.fromDegrees(relativeEncoder.getPosition());
  }

  /**
   * Get the angle of the shooter according to the absolute encoder.
   * @return The angle of the shooter as a Rotation2d.
   */
  public Rotation2d getAbsoluteAngle() {
    return RobotBase.isSimulation() ? targetAngle : Rotation2d.fromDegrees(absoluteEncoder.getPosition());
  }

  /**
   * Get absolute position of shooter without accounting for position offset.
   * @return Absolute position of shooter from 0-1.
   */
  private double getRawAbsolutePosition() {
    return (absoluteEncoder.getPosition() + absoluteEncoder.getZeroOffset()) / 360.0;
    // return RobotBase.isReal()
      // ? angleEncoder.getAbsolutePosition()
      // : armSim.getAngleRads() / (2 * Math.PI);
  }

  /**
   * Get absolute position of shooter, accounting for the position offset.
   * @return Absolute position of shooter from 0-1 accounting for position offset.
   */
  private double getAbsolutePosition() {
    return absoluteEncoder.getPosition() / 360.0;
  }

  // limits
  
  // restrict motor from moving in direction when true
  private boolean atForwardLimit() {
    return getAbsoluteAngle().getDegrees() >= forwardLimit.getDegrees();
  }

  private boolean atReverseLimit() {
    return getAbsoluteAngle().getDegrees() <= reverseLimit.getDegrees();
  }

  // disable motor completely when true
  private boolean pastForwardLimit() {
    return getAbsoluteAngle().getDegrees() > forwardLimit.getDegrees() + forwardTolerance.getDegrees();
  }
  
  private boolean pastReverseLimit() {
    return getAbsoluteAngle().getDegrees() < reverseLimit.getDegrees() - reverseTolerance.getDegrees();
  }
}
