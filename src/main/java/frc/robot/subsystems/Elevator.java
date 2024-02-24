// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Alert;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.SimpleElevatorFeedforward;
import frc.robot.utils.Alert.AlertType;

public class Elevator extends SubsystemBase {
  public static final double lowerLimitMeters = 0.0;
  public static final double lowerToleranceMeters = 0.02;
  public static final double upperLimitMeters = 0.96;
  public static final double upperToleranceMeters = 0.02;
  public static final double setpointToleranceMeters = 0.05;

  // elevator moves 2 * movement of one stage (both stages move at the same time)
  public static final double elevatorGearing = 2 * (1.0 / 9.0);
  public static final double pulleySizeMeters = 0.04;
  public static final double positionConversion = elevatorGearing * pulleySizeMeters;
  public static final double velocityConversion = positionConversion / 60.0; // convert from m/min -> m/s
  
  // TODO: intake mass (2.86) sketchy
  public static final double elevatorMassKg = 1.25 + 2.86;

  private final CANSparkMax liftMotor = new CANSparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder liftEncoder = liftMotor.getEncoder();

  private final PIDController liftPID = new PIDController(1.25, 0.0, 0.0);
  private final SimpleElevatorFeedforward liftFF = new SimpleElevatorFeedforward(0.0, 0.03, 0.0, elevatorMassKg);

  private final ElevatorSim elevatorSim = new ElevatorSim(
    DCMotor.getNEO(1),
    // wpilib expects divisor, not factor
    1.0 / elevatorGearing,
    elevatorMassKg,
    pulleySizeMeters,
    lowerLimitMeters,
    upperLimitMeters,
    true,
    lowerLimitMeters
  );

  private double targetHeight = lowerLimitMeters;
  private double lastLoopTime = Timer.getFPGATimestamp();

  private final Alert heightOutOfRange = new Alert("Elevator Height Out of Reasonable Range", AlertType.ERROR);

  private final BeaverLogger logger = new BeaverLogger();

  /** Creates a new Elevator. */
  public Elevator() {
    liftMotor.restoreFactoryDefaults();
    liftMotor.setIdleMode(IdleMode.kBrake);
    // TODO: test
    liftMotor.setInverted(true);

    liftEncoder.setPosition(lowerLimitMeters / positionConversion);

    liftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(lowerLimitMeters / positionConversion));
    liftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(upperLimitMeters / positionConversion));

    elevatorSim.update(0);
    liftFF.massKg = elevatorMassKg;

    SmartDashboard.putData("Elevator/PID", liftPID);
    SmartDashboard.putData("Elevator/FF", liftFF);

    logger.addLoggable("Elevator/FF Mass (kg)", () -> liftFF.massKg, true);
    logger.addLoggable("Elevator/Height (m)", this::getHeight, true);
    logger.addLoggable("Elevator/Velocity (mps)", this::getVelocity, true);
  }

  @Override
  public void periodic() {
    double percentOutput = liftPID.calculate(getHeight(), targetHeight) + liftFF.calculate(getVelocity());
    setMotor(percentOutput);
    logger.logAll();
  }

  @Override
  public void simulationPeriodic() {
    double currentTime = Timer.getFPGATimestamp();
    elevatorSim.update(currentTime - lastLoopTime);
    lastLoopTime = currentTime;
  }

  /**
   * Set the height of the intake the elevator will attempt to reach .
   * @param heightMeters Height to hold intake at.
   */
  public void setHeight(double heightMeters) {
    SmartDashboard.putNumber("Elevator/Desired Height Setpoint (m)", heightMeters);
    targetHeight = MathUtil.clamp(heightMeters, lowerLimitMeters, upperLimitMeters);
    SmartDashboard.putNumber("Elevator/Actual Height Setpoint (m)", targetHeight);
  }

  public boolean hasError() {
    return heightOutOfRange.isActive(); // || other alerts...
  }

  /**
   * Get the height of the elevator in meters.
   * @return Intake height in meters.
   */
  public double getHeight() {
    return RobotBase.isReal() ? liftEncoder.getPosition() * positionConversion : elevatorSim.getPositionMeters();
  }

  /**
   * Get the elevator velocity in m/s.
   * @return Elevator velocity in m/s.
   */
  public double getVelocity() {
    return RobotBase.isReal() ? liftEncoder.getVelocity() * velocityConversion : elevatorSim.getVelocityMetersPerSecond();
  }

  /**
   * Whether or not the elevator is at the requested height.
   * @return Whether or not the elevator is at the requested height.
   */
  public boolean atTargetHeight() {
    return Math.abs(targetHeight - getHeight()) <= setpointToleranceMeters;
  }

  /**
   * Set motor output, accounting for forward/backward soft limits.
   * @param percentOutput Output from -1 to 1
   */
  private void setMotor(double percentOutput) {
    SmartDashboard.putNumber("Elevator/Desired Percent Output", percentOutput);
    if (atUpperLimit()) {
      percentOutput = Math.min(percentOutput, 0);
    }
    if (atLowerLimit()) {
      percentOutput = Math.max(percentOutput, 0);
    }
    if (pastUpperLimit() || pastLowerLimit()) {
      percentOutput = 0;
      // reset PID while motor is disabled so integral doesn't build up
      liftPID.reset();
      heightOutOfRange.set(true);
    }
    SmartDashboard.putNumber("Elevator/Actual Percent Output", percentOutput);
    if (RobotBase.isReal()) {
      liftMotor.set(percentOutput);
    } else {
      if (DriverStation.isEnabled()) {
        // * 12 to convert to voltage
        elevatorSim.setInputVoltage(percentOutput * 12);
      } else {
        elevatorSim.setInputVoltage(0);
      }
    }
  }

  private boolean atUpperLimit() {
    return getHeight() >= upperLimitMeters;
  }

  private boolean atLowerLimit() {
    return getHeight() <= lowerLimitMeters;
  }

  private boolean pastUpperLimit() {
    return getHeight() > upperLimitMeters + upperToleranceMeters;
  }
  
  private boolean pastLowerLimit() {
    return getHeight() < lowerLimitMeters - lowerToleranceMeters;
  }
}
