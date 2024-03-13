// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.BeaverLogger;

public class Climber extends SubsystemBase {
  public static final Supplier<Double> raiseAxis = RobotContainer.driverController::getRightTriggerAxis;
  public static final Supplier<Double> lowerAxis = RobotContainer.driverController::getLeftTriggerAxis;
  private final CANSparkMax climberMotor = new CANSparkMax(16, MotorType.kBrushless);
  private final SparkPIDController climberPID;
  private final BeaverLogger logger = new BeaverLogger();

  public Climber() {
    climberMotor.restoreFactoryDefaults();
    climberMotor.setIdleMode(IdleMode.kBrake);
    climberMotor.setInverted(false);
    climberMotor.setSmartCurrentLimit(40);

    climberPID = climberMotor.getPIDController();
    climberPID.setSmartMotionMaxVelocity(5000, 0);
    climberPID.setSmartMotionMinOutputVelocity(0, 0);
    climberPID.setSmartMotionMaxAccel(10000, 0);
    climberPID.setSmartMotionAllowedClosedLoopError(0.1, 0);
    climberPID.setP(0.00002);
    climberPID.setI(0.000001);
    climberPID.setD(0.000005);
    climberPID.setIZone(0.3);
    climberPID.setFF(0.0002);
    climberPID.setOutputRange(-1, 1);

    logger.addLoggable("Climber/Position (Rotations)", () -> climberMotor.getEncoder().getPosition(), true);
    logger.addLoggable("Climber/Applied Output (-1-1)", climberMotor::getAppliedOutput, true);
  }

  @Override
  public void periodic() {
    logger.logAll();
  }

  /**
   * Set the climber motor to 'Climber::raiseAxis'.
   * Does not restrict movement at climber limits.
   * @return Command to continually set the climber to 'raiseAxis'.
   */
  public RunCommand raise() {
    return new RunCommand(() -> climberMotor.set(raiseAxis.get()), this);
  }

  /**
   * Set the climber motor to 'Climber::lowerAxis'.
   * Does not restrict movement at climber limits.
   * @return Command to continually set the climber to 'lowerAxis'.
   */
  public RunCommand lower() {
    return new RunCommand(() -> climberMotor.set(-lowerAxis.get()), this);
  }

  /**
   * Hold the climber's current position using PID.
   * @return An InstantCommand that sets the reference of the climber PID to its current position.
   */
  public InstantCommand hold() {
    return new InstantCommand(() -> climberPID.setReference(climberMotor.getEncoder().getPosition(), CANSparkMax.ControlType.kSmartMotion), this);
  }

  public InstantCommand stop() {
    return new InstantCommand(climberMotor::disable, this);
  }
}