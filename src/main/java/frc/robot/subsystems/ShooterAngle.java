// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED.BlinkMode;
import frc.robot.subsystems.LED.Pattern;
import frc.robot.utils.Alert;
import frc.robot.utils.ArmFeedforward;
import frc.robot.utils.Alert.AlertType;
import frc.robot.utils.BeaverLogger;
// import frc.robot.utils.TunableNumber;
import frc.robot.utils.TunableNumber;

// TODO: add back simulation
// TODO: javadocs
// TODO: override does nothing
// TODO: 'absSetCounter should be removed'

public class ShooterAngle extends SubsystemBase {
  // ** Angle Direction: +ve = Forward, towards elevator. **
  public static final Rotation2d forwardLimit = Rotation2d.fromDegrees(70);
  public static final Rotation2d forwardTolerance = Rotation2d.fromDegrees(3);
  public static final Rotation2d reverseLimit = Rotation2d.fromDegrees(8.9);
  public static final Rotation2d reverseTolerance = Rotation2d.fromDegrees(1);
  public static final Rotation2d setpointTolerance = Rotation2d.fromDegrees(1.5);
  private double angleFudge = 0.0;

  // Model constants
  public static final double massKg = 6.5;
  public static final double comDistanceFromPivotMeters = 0.20;
  public static final double comAngleFromForwardDegrees = 12;
  public static final double armLengthMeters = 0.35;

  // measure at 90 degrees
  public static final double absoluteEncoderOffset = 0.598 - (90.0 / 360.0);
  // 15:22 chain and 1:81 planetary
  public static final double positionConversionFactor = (15.0 / 22.0) * (1.0 / 81.0) * 360; // deg
  public static final double velocityConversionFactor = positionConversionFactor / 60; // deg/s
  
  private final TunableNumber anglePGain = new TunableNumber("Shooter Angle P", 0.01);
  private final TunableNumber angleIGain = new TunableNumber("Shooter Angle I", 0.1);
  private final TunableNumber angleDGain = new TunableNumber("Shooter Angle D", 0.0);
  private final TunableNumber angleKSGain = new TunableNumber("Shooter Angle kS", 0.0);
  private final TunableNumber angleKGGain = new TunableNumber("Shooter Angle kG", 0.005);
  private final TunableNumber angleKVGain = new TunableNumber("Shooter Angle kV", 0.001);

  private double kv = 0.0;

  private final CANSparkMax angleMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder relativeEncoder = angleMotor.getEncoder(); // unit: DEGREES
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(5);

  
  // p = volts/degree of error
  // when you tune these, REMEMBER THERE IS A VOLTAGE LIMIT ON THE MOTOR!
  public final ProfiledPIDController anglePID = new ProfiledPIDController(0.02, 0.0, 0.0, new Constraints(280, 700));
  public final ArmFeedforward angleFF = new ArmFeedforward(0.0, 0.005, comDistanceFromPivotMeters, comAngleFromForwardDegrees, massKg, "Shooter/Angle");

  
  private double lastLoopTime = Timer.getFPGATimestamp();
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

    absoluteEncoder.setPositionOffset(absoluteEncoderOffset);
    relativeEncoder.setPositionConversionFactor(positionConversionFactor);
    relativeEncoder.setVelocityConversionFactor(velocityConversionFactor);
    SmartDashboard.putNumber("AngleFudge", angleFudge);

    anglePID.setIZone(4.0);

    anglePID.setGoal(targetAngle.getDegrees());

    SmartDashboard.putData("Shooter/Angle/PID", anglePID);
    SmartDashboard.putData("Shooter/Angle/FF", angleFF);

    // armSim.update(0); // setup simulation before periodic() runs for the first time

    SmartDashboard.putBoolean("Overrides/Shooter Angle", false);
    logger.addLoggable("Shooter/Angle/Raw Absolute Position (0-1)", this::getRawAbsolutePosition, true);
    logger.addLoggable("Shooter/Angle/Absolute Position (0-1)", this::getAbsolutePosition, true);
    logger.addLoggable("Shooter/Angle/Absolute Angle (deg)", () -> getAbsoluteAngle().getDegrees(), true);
    logger.addLoggable("Shooter/Angle/Relative Angle (deg)", () -> getRelativeAngle().getDegrees(), true);
    logger.addLoggable("Shooter/Angle/Applied Output (-1-1)", angleMotor::getAppliedOutput, true);
    logger.addLoggable("Shooter/Angle/Motion Profile Setpoint (Deg)", () -> anglePID.getSetpoint().position, true);
  }

  @Override
  public void periodic() {
    //test
    // setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("Test/Shooter Target Angle (Deg)", 0)));
    if(anglePID.getSetpoint().velocity != 0.0 || Math.abs(anglePID.getSetpoint().position-forwardLimit.getDegrees())<setpointTolerance.getDegrees() || DriverStation.isDisabled()){
      // don't calculate I on ramp-up
      // or near rest
      anglePID.reset(anglePID.getSetpoint());
    }
    double voltage = anglePID.calculate(getAbsoluteAngle().getDegrees(), targetAngle.getDegrees()) + angleFF.calculate(Rotation2d.fromDegrees(getAbsoluteAngle().getDegrees()));
    
    double velocity_FF = MathUtil.clamp(kv*anglePID.getSetpoint().velocity, -0.4, 0.4);
    
    voltage += velocity_FF;
    SmartDashboard.putNumber("Shooter/Angle/VelocitySetpoint", anglePID.getSetpoint().velocity);
    SmartDashboard.putNumber("Shooter/Angle/VelocityFF", velocity_FF);
    SmartDashboard.putNumber("Shooter/Angle/Calculated Voltage", voltage);
    setMotor(voltage);
    // set alerts
    if (pastForwardLimit() || pastReverseLimit()) {
      absoluteAngleOutOfRange.set(true);
      stop(); // in case setAngle/setMotor isn't being called
    } else {
      absoluteAngleOutOfRange.set(false);
    }

    // check tunable numbers
    if (anglePGain.hasChanged(hashCode()) || angleDGain.hasChanged(hashCode()) || angleIGain.hasChanged(hashCode()) || angleKGGain.hasChanged(hashCode()) || angleKSGain.hasChanged(hashCode()) || angleKVGain.hasChanged(hashCode())) {
      anglePID.setP(anglePGain.get());
      anglePID.setI(angleIGain.get());
      anglePID.setD(angleDGain.get()); // this doesn't work properly
      angleFF.kG = angleKGGain.get();
      angleFF.kS = angleKSGain.get();
      kv = angleKVGain.get();
    }

    // set sim mechanism here (using % output from Spark)

    SmartDashboard.putBoolean("Shooter/Angle/At Target", atTarget());
    if(atTarget()){
      RobotContainer.led.setSegmentPattern("angle", Pattern.INTAKE_NOTE, BlinkMode.SOLID);
    }else{
      RobotContainer.led.setSegmentPattern("angle", Pattern.RED, BlinkMode.SOLID);
    }
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
    angle = Rotation2d.fromDegrees(angle.getDegrees() + SmartDashboard.getNumber("Angle Fudge", 0.0));
    targetAngle = angle; // used to check 'atTarget()' and calculate PIDF
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
    // there should be no reason we ever drive our shooter down more than 10%... Gravity will do it for us
    percentOutput = MathUtil.clamp(percentOutput, -0.1, 1);
    SmartDashboard.putNumber("Shooter/Angle/Target Output Voltage", percentOutput*12.0);
    SmartDashboard.putNumber("Shooter/Angle/Output duty cycle", angleMotor.getAppliedOutput());
    SmartDashboard.putNumber("Shooter/Angle/BusVoltage", angleMotor.getBusVoltage());
    angleMotor.setVoltage(percentOutput*12.0);
  }

  public void stop() {
    angleMotor.set(0);
    targetAngle = reverseLimit;
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
    return Rotation2d.fromDegrees(RobotBase.isSimulation() ? targetAngle.getDegrees() : relativeEncoder.getPosition());
  }

  /**
   * Get the angle of the shooter according to the absolute encoder.
   * @return The angle of the shooter as a Rotation2d.
   */
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRotations(RobotBase.isSimulation() ? targetAngle.getRotations() : getAbsolutePosition());
  }

  /**
   * Get absolute position of shooter without accounting for position offset.
   * @return Absolute position of shooter from 0-1.
   */
  private double getRawAbsolutePosition() {
    return absoluteEncoder.getAbsolutePosition();
    // return RobotBase.isReal()
      // ? angleEncoder.getAbsolutePosition()
      // : armSim.getAngleRads() / (2 * Math.PI);
  }

  /**
   * Get absolute position of shooter, accounting for the position offset.
   * @return Absolute position of shooter from 0-1 accounting for position offset.
   */
  private double getAbsolutePosition() {
    double position = absoluteEncoder.getAbsolutePosition() - absoluteEncoder.getPositionOffset();
    // double position = RobotBase.isReal()
    //   ? angleEncoder.getAbsolutePosition() - angleEncoder.getPositionOffset()
    //   : armSim.getAngleRads() / (2 * Math.PI);
    // if negative, roll over to positive value
    if (position < 0) {
      position = position + 1;
    }
    return position;
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
