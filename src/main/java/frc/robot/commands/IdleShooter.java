// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFlywheel.SpinType;

public class IdleShooter extends InstantCommand {
  /**
   * Set the shooter angle to the minimum angle and the flywheel to 0 then end.
   */
  public IdleShooter() {
    addRequirements(RobotContainer.shooterAngle, RobotContainer.shooterFlywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shooterAngle.stop();
    RobotContainer.shooterFlywheel.setSpeed(0, SpinType.disable);
  }
}
