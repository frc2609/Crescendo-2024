// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  private final CANSparkMax climberMotor = new CANSparkMax(16, MotorType.kBrushless);
  private final SparkPIDController climberPID;

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
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/Position (Rotations)", climberMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber/Applied Output (-1-1)", climberMotor.getAppliedOutput());
  }

  public Command Raise() {
    return new RunCommand(() -> climberMotor.set(RobotContainer.driverController.getLeftTriggerAxis()), this);
  }

  public Command Lower() {
    return new RunCommand(() -> climberMotor.set(-RobotContainer.driverController.getRightTriggerAxis()), this);
  }

  public Command Stop() {
    return new InstantCommand(() -> climberPID.setReference(climberMotor.getEncoder().getPosition(), CANSparkMax.ControlType.kSmartMotion), this);
  }
}