// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFlywheel.SpinType;

public class PassthroughNote extends Command {
  /** Creates a new PassthroughNote. */
  public PassthroughNote() {
    // prevent shooter angle from moving while this is running
    addRequirements(RobotContainer.intake, RobotContainer.shooterFlywheel, RobotContainer.shooterAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.setMotor(1);
    RobotContainer.shooterFlywheel.setSpeed(2000, SpinType.disable);
  }

  @Override
  public void end(boolean interrupted) {
    // RobotContainer.intake.setMotor(0);
    // RobotContainer.shooterFlywheel.coast();
  }
}
