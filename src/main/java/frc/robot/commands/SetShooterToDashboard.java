// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFlywheel.SpinType;

/**
 * Set the shooter to the angle, RPM, and spin specified through NetworkTables.
 */
public class SetShooterToDashboard extends Command {
  /** Creates a new SetShooterToDashboard. */
  public SetShooterToDashboard() {
    addRequirements(RobotContainer.shooterAngle, RobotContainer.shooterFlywheel);
  }

  @Override
  public void execute() {
    RobotContainer.shooterAngle.setAngle(new Rotation2d(Math.toRadians(SmartDashboard.getNumber("Shooter Angle Setpoint", 20.0))));
    
    RobotContainer.shooterFlywheel.setSpeed(
      SmartDashboard.getNumber("RPM Setpoint", 3500.0),
      SmartDashboard.getBoolean("Spin Enabled", true) ? SpinType.slowRightMotor : SpinType.disable
    );
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterAngle.stop();
    RobotContainer.shooterFlywheel.coast();
  }
}
