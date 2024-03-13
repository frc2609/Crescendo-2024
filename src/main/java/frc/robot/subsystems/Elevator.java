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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Alert;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.SimpleElevatorFeedforward;
import frc.robot.utils.TunableNumber;
import frc.robot.utils.Alert.AlertType;

public class Elevator extends SubsystemBase {
  public static final double lowerLimitMeters = 0.0;
  public static final double lowerToleranceMeters = 0.02;
  public static final double upperLimitMeters = 0.93;
  public static final double upperToleranceMeters = 0.02;
  public static final double setpointToleranceMeters = 0.05;

  // elevator moves 2 * movement of one stage (both stages move at the same time)
  public static final double elevatorGearing = 2 * (1.0 / 9.0);
  public static final double pulleySizeMeters = 0.0362;
  public static final double positionConversion = elevatorGearing * pulleySizeMeters * Math.PI;
  public static final double velocityConversion = positionConversion / 60.0; // convert from m/min -> m/s
  
  public static final double elevatorMassKg = 4.11; // educated guess
  public static final TunableNumber motorVoltageLimit = new TunableNumber("Elevator/Voltage Limit (V)", 12);

  private final CANSparkMax liftMotor = new CANSparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder liftEncoder = liftMotor.getEncoder();

  // p = volts/meter of error
  private final ProfiledPIDController liftPID = new ProfiledPIDController(12, 0, 0, new Constraints(2, 12));
  private final SimpleElevatorFeedforward liftFF = new SimpleElevatorFeedforward(0.0, 0.1, 0.0, elevatorMassKg);

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
    liftMotor.setInverted(true);

    liftEncoder.setPosition(lowerLimitMeters / positionConversion);

    liftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(lowerLimitMeters / positionConversion));
    liftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(upperLimitMeters / positionConversion));

    elevatorSim.update(0);

    SmartDashboard.putData("Elevator/PID", liftPID);
    SmartDashboard.putData("Elevator/FF", liftFF);

    logger.addLoggable("Elevator/Height (m)", this::getHeight, true);
    logger.addLoggable("Elevator/Profiled PID Setpoint (m)", () -> liftPID.getSetpoint().position, true);
    logger.addLoggable("Elevator/Velocity (mps)", this::getVelocity, true);
  }

  @Override
  public void periodic() {
    // disable the elevator motor when it is told to go to the bottom *and already at the bottom*
    // disabling it when the target height is 0 would cause to fall slowly (takes much longer)
    if ((targetHeight == lowerLimitMeters) && atTargetHeight()) {
      setMotor(0);
    } else {
      double voltage = liftPID.calculate(getHeight(), targetHeight) + liftFF.calculate(getVelocity());
      setMotor(voltage);
    }

    SmartDashboard.putBoolean("Elevator/At Target Height", atTargetHeight());
    logger.logAll();
  }

  @Override
  public void simulationPeriodic() {
    double currentTime = Timer.getFPGATimestamp();
    elevatorSim.update(currentTime - lastLoopTime);
    lastLoopTime = currentTime;
  }

  public void stop() {
    liftPID.reset(getHeight());
    liftMotor.disable();
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
   * @param voltage Voltage from -12 to 12
   */
  private void setMotor(double voltage) {
    SmartDashboard.putNumber("Elevator/Desired Voltage", voltage);
    if (atUpperLimit()) {
      voltage = Math.min(voltage, 0);
    }
    if (atLowerLimit()) {
      voltage = Math.max(voltage, 0);
    }
    if (pastUpperLimit() || pastLowerLimit()) {
      voltage = 0;
      // reset PID while motor is disabled so integral doesn't build up
      liftPID.reset(getHeight());
      heightOutOfRange.set(true);
    }
    voltage = MathUtil.clamp(voltage, -motorVoltageLimit.get(), motorVoltageLimit.get());
    SmartDashboard.putNumber("Elevator/Actual Voltage", voltage);
    if (RobotBase.isReal()) {
      liftMotor.setVoltage(voltage);
    } else {
      if (DriverStation.isEnabled()) {
        elevatorSim.setInputVoltage(voltage);
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
