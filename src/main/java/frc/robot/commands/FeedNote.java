// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/** Feed the note to the shooter. Ends after a timer expires. */
public class FeedNote extends Command {
  /** Creates a new FeedNote. */
  public FeedNote() {
    addRequirements(RobotContainer.intake);
    withTimeout(0.4);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.setMotor(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setMotor(0);
    // tell Visualizer that we no longer have the note
    RobotContainer.intake.noteHeld = false;
  }
}
