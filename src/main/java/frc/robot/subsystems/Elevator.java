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
  /* Height intake can reach before elevator stage engages. */
  public static final double intakeMaxHeightMeters = 0.48;
  /* Height elevator stage can reach. */
  public static final double stageMaxHeightMeters = 0.49;

  public static final double lowerLimitMeters = 0.0;
  public static final double lowerToleranceMeters = 0.02;
  // intake reaches max height of elevator stage + max height of intake within elevator stage
  public static final double upperLimitMeters = intakeMaxHeightMeters + stageMaxHeightMeters;
  public static final double upperToleranceMeters = 0.02;

  // TODO: set these too...
  public static final double intakePositionMeters = 0.0;
  public static final double ampPositionMeters = 0.0;
  public static final double trapPositionMeters = 0.0;

  public static final double elevatorGearing = (1.0 / 9.0);
  public static final double pulleySizeMeters = 0.04;
  public static final double positionConversion = elevatorGearing * pulleySizeMeters;
  
  // TODO: intake mass sketchy
  public static final double intakeMassKg = 2.86;
  public static final double elevatorMassKg = 1.25;

  private final CANSparkMax liftMotor = new CANSparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder liftEncoder = liftMotor.getEncoder();

  private final PIDController liftPID = new PIDController(1.25, 0.0, 0.0);
  private final SimpleElevatorFeedforward liftFF = new SimpleElevatorFeedforward(0.0, 0.03, 0.0, intakeMassKg);

  private final ElevatorSim intakeSim = new ElevatorSim(
    DCMotor.getNEO(1),
    // wpilib expects divisor, not factor
    1.0 / elevatorGearing,
    intakeMassKg,
    pulleySizeMeters,
    lowerLimitMeters,
    intakeMaxHeightMeters,
    true,
    lowerLimitMeters
  );

  private final ElevatorSim stageSim = new ElevatorSim(
    DCMotor.getNEO(1),
    // wpilib expects divisor, not factor
    1.0 / elevatorGearing,
    intakeMassKg + elevatorMassKg,
    pulleySizeMeters,
    lowerLimitMeters,
    stageMaxHeightMeters,
    true,
    lowerLimitMeters
  );

  private double targetHeight = lowerLimitMeters;
  private double lastLoopTime = Timer.getFPGATimestamp();

  private final Alert heightOutOfRange = new Alert("Elevator Height Out of Reasonable Range", AlertType.ERROR);

  private final BeaverLogger logger = new BeaverLogger();

  /** Creates a new Elevator. */
  public Elevator() {
    liftMotor.setIdleMode(IdleMode.kBrake);
    // TODO: test
    liftMotor.setInverted(true);

    liftEncoder.setPositionConversionFactor(positionConversion);
    // convert to m/s from m/min
    liftEncoder.setVelocityConversionFactor(positionConversion / 60.0);
    liftEncoder.setPosition(lowerLimitMeters);

    liftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)lowerLimitMeters);
    liftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)upperLimitMeters);

    intakeSim.update(0);
    stageSim.update(0);

    SmartDashboard.putData("Elevator/PID", liftPID);
    SmartDashboard.putData("Elevator/FF", liftFF);

    logger.addLoggable("Elevator/FF Mass (kg)", () -> liftFF.massKg, true);
    logger.addLoggable("Elevator/Overall Height (m)", this::getHeight, true);
    logger.addLoggable("Elevator/Stage Height (m)", this::getElevatorStageHeight, true);
    logger.addLoggable("Elevator/Velocity (mps)", this::getVelocity, true);
  }

  @Override
  public void periodic() {
    // if elevator is ~= at the bottom, the motor only feels the force from the mass of the intake
    if (getElevatorStageHeight() <= lowerLimitMeters + lowerToleranceMeters) {
      liftFF.massKg = intakeMassKg;
    } else {
      liftFF.massKg = intakeMassKg + elevatorMassKg;
    }

    double percentOutput = liftPID.calculate(getHeight(), targetHeight) + liftFF.calculate(getVelocity());
    setMotor(percentOutput);
    logger.logAll();
  }

  @Override
  public void simulationPeriodic() {
    double currentTime = Timer.getFPGATimestamp();
    intakeSim.update(currentTime - lastLoopTime);
    stageSim.update(currentTime - lastLoopTime);
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

  // TODO: simulation compatibility for below 3 functions
  /**
   * Get the height of the elevator stage in meters.
   * The elevator only moves once the intake reaches its max height.
   * @return Elevator height in meters.
   */
  public double getElevatorStageHeight() {
    // elevator doesn't move until intake reaches top of elevator stage
    return RobotBase.isReal() ? Math.max(getHeight() - intakeMaxHeightMeters, 0) : stageSim.getPositionMeters();
  }

  /**
   * Get the height of the intake in meters.
   * The intake moves independently from the elevator stage until it reaches its max height, then
   * it moves along with the elevator stage.
   * @return Intake height in meters.
   */
  public double getHeight() {
    return RobotBase.isReal() ? liftEncoder.getPosition() : intakeSim.getPositionMeters() + stageSim.getPositionMeters();
  }

  /**
   * Get the elevator velocity in m/s.
   * @return Elevator velocity in m/s.
   */
  public double getVelocity() {
    // TODO: sim support (complex, not doing now)
    return liftEncoder.getVelocity();
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
        intakeSim.setInputVoltage(percentOutput * 12);
        stageSim.setInputVoltage(intakeAtMaxHeight() ? percentOutput * 12 : 0); // convert to voltage
      } else {
        intakeSim.setInputVoltage(0);
        stageSim.setInputVoltage(0);
      }
    }
  }

  private boolean intakeAtMaxHeight() {
    return getHeight() >= intakeMaxHeightMeters;
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
