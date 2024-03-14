// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Alert;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.Alert.AlertType;

public class Elevator extends SubsystemBase {
  public static final double lowerLimitMeters = 0.0;
  public static final double lowerToleranceMeters = 0.02;
  public static final double upperLimitMeters = 0.95;
  public static final double upperToleranceMeters = 0.02;
  public static final double setpointToleranceMeters = 0.05;

  // elevator moves 2 * movement of one stage (both stages move at the same time)
  public static final double elevatorGearing = 2 * (1.0 / 9.0);
  public static final double pulleySizeMeters = 0.0362;
  public static final double positionConversion = elevatorGearing * pulleySizeMeters * Math.PI;
  public static final double velocityConversion = positionConversion / 60.0; // convert from m/min -> m/s
  
  private final CANSparkMax liftMotor = new CANSparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder liftEncoder = liftMotor.getEncoder();
  private final SparkPIDController liftPID = liftMotor.getPIDController();

  public static final double elevatorMassKg = 4.11; // educated guess
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
    liftMotor.setSmartCurrentLimit(40);

    liftPID.setSmartMotionMaxVelocity(5000, 0);
    liftPID.setSmartMotionMinOutputVelocity(0, 0);
    liftPID.setSmartMotionMaxAccel(10000, 0);
    liftPID.setSmartMotionAllowedClosedLoopError(0.1, 0);
    liftPID.setP(0.00002);
    liftPID.setI(0.000001);
    liftPID.setD(0.000005);
    liftPID.setIZone(0.3);
    liftPID.setFF(0.0002);
    liftPID.setOutputRange(-1, 1);

    liftEncoder.setPosition(lowerLimitMeters / positionConversion);

    liftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)(lowerLimitMeters / positionConversion));
    liftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)(upperLimitMeters / positionConversion));

    elevatorSim.update(0);

    SmartDashboard.putBoolean("Overrides/Elevator", false);
    logger.addLoggable("Elevator/Height (m)", this::getHeight, true);
    logger.addLoggable("Elevator/Velocity (mps)", this::getVelocity, true);
    logger.addLoggable("Elevator/Applied Output (-1-1)", liftMotor::getAppliedOutput, true);
  }

  @Override
  public void periodic() {
    if (pastLowerLimit() || pastUpperLimit()) {
      heightOutOfRange.set(true);
      stop(); // in case setAngle/setMotor isn't being called
    } else {
      heightOutOfRange.set(false);
    }

    // set sim mechanism here (using % output from Spark)
    // setInputVoltage according to PID when PID running only (i.e. % out != 0)

    SmartDashboard.putBoolean("Elevator/At Target", atTarget());
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
   * @param height Height to hold intake at.
   */
  public void setHeight(double height) {
    if (hasError() && !errorOverridden()) {
      stop();
      return;
    }
    // clamp height to allowable setpoints
    SmartDashboard.putNumber("Elevator/Desired Height Setpoint (m)", height);
    height = MathUtil.clamp(height, lowerLimitMeters, upperLimitMeters);
    SmartDashboard.putNumber("Elevator/Actual Height Setpoint (m)", targetHeight);
    // set angle
    targetHeight = height; // used to check 'atTarget()'
    liftPID.setReference(height / positionConversion, ControlType.kSmartMotion);
  }

  /**
   * Set motor output, accounting for forward/backward soft limits.
   * @param percentOutput Voltage from -12 to 12
   */
  public void setMotor(double percentOutput) {
    SmartDashboard.putNumber("Elevator/Desired Percent Output", percentOutput);
    if (!errorOverridden()) {
      if (atUpperLimit()) {
        percentOutput = Math.min(percentOutput, 0);
      }
      if (atLowerLimit()) {
        percentOutput = Math.max(percentOutput, 0);
      }
      if (hasError()) {
        percentOutput = 0;
      }
    }
    percentOutput = MathUtil.clamp(percentOutput, -1, 1);
    SmartDashboard.putNumber("Elevator/Actual Percent Output", percentOutput);
    liftMotor.set(percentOutput);
    // simulation
    if (RobotBase.isSimulation()) {
      if (DriverStation.isEnabled()) {
        elevatorSim.setInputVoltage(percentOutput);
      } else {
        elevatorSim.setInputVoltage(0);
      }
    }
  }

  public void stop() {
    liftMotor.disable();
  }

  /**
   * Whether or not the elevator is at the requested height.
   * @return Whether or not the elevator is at the requested height.
   */
  public boolean atTarget() {
    return Math.abs(targetHeight - getHeight()) <= setpointToleranceMeters;
  }

  public boolean hasError() {
    return heightOutOfRange.isActive(); // || other alerts...
  }

  public boolean errorOverridden() {
    return SmartDashboard.getBoolean("Overrides/Elevator", false);
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

  // limits

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
