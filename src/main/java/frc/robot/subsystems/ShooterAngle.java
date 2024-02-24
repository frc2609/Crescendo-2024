// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Alert;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.Alert.AlertType;

public class ShooterAngle extends SubsystemBase {
  // forward = +ve, towards elevator
  // motor is prevented from moving in one direction when a limit is reached
  public static final Rotation2d forwardLimit = Rotation2d.fromDegrees(55);
  // motor is completely disabled if limit + tolerance (or - tolerance if reverse) is exceeded
  public static final Rotation2d forwardTolerance = Rotation2d.fromDegrees(3);
  public static final Rotation2d reverseLimit = Rotation2d.fromDegrees(8.0);
  public static final Rotation2d reverseTolerance = Rotation2d.fromDegrees(0.25);

  public static final double motorPercentOutputLimit = 0.4;
  // measure at 90 degrees
  public static final double angleEncoderOffset = 0.775 - (90.0 / 360.0);
  public static final double massKg = 7.5;
  public static final double comDistanceFromPivotMeters = 0.25; // estimate
  public static final double armLengthMeters = 0.35;
  // 15:22 chain and 1:81 planetary
  public static final double motorToShaftRatio = (22.0 / 15.0) * 81.0;
 
  public final CANSparkMax angleMotor = new CANSparkMax(14, MotorType.kBrushless);
  public final DutyCycleEncoder angleEncoder = new DutyCycleEncoder(9);
  public final SingleJointedArmSim armSim = new SingleJointedArmSim(
    DCMotor.getNeo550(1),
    motorToShaftRatio,
    SingleJointedArmSim.estimateMOI(armLengthMeters, massKg),
    // massKg * comDistanceFromPivot * comDistanceFromPivot,
    armLengthMeters,
    reverseLimit.getRadians(),
    Math.toRadians(85), // can physically travel further than forward soft limit
    true,
    reverseLimit.getRadians()
  );

  // TODO: Tune these for 550
  public final PIDController anglePID = new PIDController(0.0, 0, 0.0);
  public final ArmFeedforward angleFF = new ArmFeedforward(0.0, 0.006, comDistanceFromPivotMeters, massKg, "Shooter/Angle");

  // assumed to be at lower hard stop (natural resting place)
  private Rotation2d targetAngle = reverseLimit;
  private double lastLoopTime = Timer.getFPGATimestamp();

  private final Alert angleOutOfRange = new Alert("Shooter Angle Out of Reasonable Range", AlertType.ERROR);
  
  private final BeaverLogger logger = new BeaverLogger();

  /** Creates a new ShooterAngle. */
  public ShooterAngle() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setInverted(true);
    angleMotor.getEncoder().setPositionConversionFactor(1);
    angleMotor.getEncoder().setPosition(0.0);
    
    angleEncoder.setPositionOffset(angleEncoderOffset);
    armSim.update(0); // setup simulation before periodic() runs for the first time

    SmartDashboard.putData("Shooter/Angle/PID", anglePID);
    SmartDashboard.putData("Shooter/Angle/FF", angleFF);

    logger.addLoggable("Shooter/Angle/Raw Absolute (0-1)", this::getRawAbsolutePosition, true);
    logger.addLoggable("Shooter/Angle/Absolute (0-1)", this::getAbsolutePosition, true);
    logger.addLoggable("Shooter/Angle/Current (Deg)", () -> getAngle().getDegrees(), true);
    logger.addLoggable("Shooter/Angle/Target (Deg)", () -> targetAngle.getDegrees(), true);
    logger.addLoggable("Shooter/Angle/NEO Position (Deg)", () -> getSparkAngle().getDegrees(), true);
  }

  @Override
  public void periodic() {
    double percentOutput = anglePID.calculate(getAngle().getDegrees(), targetAngle.getDegrees()) + angleFF.calculate(getAngle());
    setMotor(percentOutput);
    logger.logAll();
  }

  @Override
  public void simulationPeriodic() {
    double currentTime = Timer.getFPGATimestamp();
    armSim.update(currentTime - lastLoopTime);
    lastLoopTime = currentTime;
  }

  public boolean hasError() {
    return angleOutOfRange.isActive(); // || other alerts...
  }

  /**
   * Set the angle the shooter will attempt to hold.
   * @param angle Angle to hold shooter at.
   */
  public void setAngle(Rotation2d angle) {
    SmartDashboard.putNumber("Shooter/Angle/Desired Setpoint (Deg)", angle.getDegrees());
    targetAngle = Rotation2d.fromDegrees(MathUtil.clamp(angle.getDegrees(), reverseLimit.getDegrees(), forwardLimit.getDegrees()));
    SmartDashboard.putNumber("Shooter/Angle/Actual Setpoint (Deg)", targetAngle.getDegrees());
  }

  /**
   * Get the angle of the shooter according to the absolute encoder.
   * @return The angle of the shooter as a Rotation2d, always positive.
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getAbsolutePosition());
  }

  /**
   * Get the angle of the shooter according to the shooter motor's relative encoder.
   * Does nothing in simulation.
   * @return The angle of the shooter as a Rotation2d. Not guaranteed to be positive.
   */
  public Rotation2d getSparkAngle() {
    return Rotation2d.fromDegrees(
      ((angleMotor.getEncoder().getPosition() / motorToShaftRatio) * 360.0) + reverseLimit.getDegrees()
    );
  }

  /**
   * Set motor output, accounting for forward/backward soft limits and percentOutput limit.
   * @param percentOutput Output from -1 to 1
   */
  private void setMotor(double percentOutput) {
    SmartDashboard.putNumber("Shooter/Angle/Desired Percent Output", percentOutput);
    if (atForwardLimit()) {
      percentOutput = Math.min(percentOutput, 0);
    }
    if (atReverseLimit()) {
      percentOutput = Math.max(percentOutput, 0);
    }
    if (pastForwardLimit() || pastReverseLimit()) {
      percentOutput = 0;
      // reset PID while motor is disabled so integral doesn't build up
      anglePID.reset();
      angleOutOfRange.set(true);
    }
    percentOutput = MathUtil.clamp(percentOutput, -motorPercentOutputLimit, motorPercentOutputLimit);
    SmartDashboard.putNumber("Shooter/Angle/Actual Percent Output", percentOutput);
    if (RobotBase.isReal()) {
      // angleMotor.set(percentOutput);
    } else {
      if (DriverStation.isEnabled()) {
        armSim.setInputVoltage(percentOutput * 12); // convert to voltage
      } else {
        armSim.setInputVoltage(0);
      }
    }
  }

  /**
   * Get absolute position of shooter without accounting for position offset.
   * @return Absolute position of shooter.
   */
  private double getRawAbsolutePosition() {
    return RobotBase.isReal()
      ? angleEncoder.getAbsolutePosition()
      : armSim.getAngleRads() / (2 * Math.PI);
  }

  /**
   * Get absolute position of shooter, accounting for the position offset.
   * @return Absolute position of shooter - position offset.
   */
  private double getAbsolutePosition() {
    double position = RobotBase.isReal()
      ? angleEncoder.getAbsolutePosition() - angleEncoder.getPositionOffset()
      : armSim.getAngleRads() / (2 * Math.PI);
    // if negative, roll over to positive value
    if (position < 0) {
      position = 1 - position;
    }
    return position;
  }

  private boolean atForwardLimit() {
    return getAngle().getDegrees() >= forwardLimit.getDegrees();
  }

  private boolean atReverseLimit() {
    return getAngle().getDegrees() <= reverseLimit.getDegrees();
  }

  private boolean pastForwardLimit() {
    return getAngle().getDegrees() > forwardLimit.getDegrees() + forwardTolerance.getDegrees();
  }
  
  private boolean pastReverseLimit() {
    return getAngle().getDegrees() < reverseLimit.getDegrees() - reverseTolerance.getDegrees();
  }
}
