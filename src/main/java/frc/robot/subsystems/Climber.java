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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.BeaverLogger;

public class Climber extends SubsystemBase {
  private final CANSparkMax climberMotor = new CANSparkMax(16, MotorType.kBrushless);
  private final SparkPIDController climberPID;
  private final BeaverLogger logger = new BeaverLogger();

  public Climber() {
    climberMotor.restoreFactoryDefaults();
    climberMotor.setIdleMode(IdleMode.kBrake);
    climberMotor.setInverted(true);
    climberMotor.setSmartCurrentLimit(40);

    climberPID = climberMotor.getPIDController();
    climberPID.setSmartMotionMaxVelocity(5000, 0);
    climberPID.setSmartMotionMinOutputVelocity(0, 0);
    climberPID.setSmartMotionMaxAccel(5000, 0);
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
   * Move the climber at the speed specified by 'percentOutput'.
   * Does not restrict movement at climber limits.
   * <p>Positive speeds raise the climber (lowering the robot) and negative speeds lower it
   * (raising the robot).
   * @param percentOutput A supplier used to set the speed of the motor (e.g. a controller axis).
   * @return A command to move the climber at the speed specified by the 'percentOutput' supplier.
   */
  public RunCommand getMove(Supplier<Double> percentOutput) {
    return new RunCommand(() -> setMotor(percentOutput.get()), this);
  }

  /**
   * Hold the climber's current position using PID.
   * @return A command that sets the reference of the climber PID to its current position.
   */
  public Command getHold() {
    return new InstantCommand(() -> setReference(climberMotor.getEncoder().getPosition()), this);
  }

  /**
   * Stop the climber.
   * @return A command that stops the climber.
   */
  public Command getStop() {
    return new InstantCommand(this::stop).ignoringDisable(true);
  }

  public void stop() {
    SmartDashboard.putNumber("Climber/Desired Percent Output", 0);
    climberMotor.disable();
  }

  // wrappers for setMotor/setReference that log input values

  private void setMotor(double percentOutput) {
    SmartDashboard.putNumber("Climber/Desired Percent Output", percentOutput);
    climberMotor.set(percentOutput);
  }

  private void setReference(double position) {
    SmartDashboard.putNumber("Climber/Hold Position (Rotations)", position);
    climberPID.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }
}