// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/**
 * Feed the note to the shooter when the shooter angle and flywheel are at their setpoint for a
 * certain amount of loop cycles.
 */
public class FeedWhenReady extends SequentialCommandGroup {
  public final double maxCounter = 3;
  private double counter = 0;

  /** Creates a new FeedWhenReady. */
  public FeedWhenReady() {
    addCommands(
      Commands.run(
        this::checkCounter
      ).until(() -> counter >= maxCounter),
      RobotContainer.intake.getFeedNote()
    );
  }

  private void checkCounter() {
    // whether this command runs before or after setting the shooter angle/flywheel isn't defined
    // this is a race condition (bad) since this command could check atTargetAngle and atSetSpeed
    // before they are actually set, but the loop counter circumvents this :)
    if (RobotContainer.shooterAngle.atTargetAngle() && RobotContainer.shooterFlywheel.atSetSpeed()) {
      counter++;
    } else {
      counter = 0;
    }
    SmartDashboard.putNumber("Commands/FeedWhenReady/Counter", counter);
  }
}
