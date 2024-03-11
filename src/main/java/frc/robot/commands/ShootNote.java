// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFlywheel.SpinType;


public class ShootNote extends SequentialCommandGroup {
  /**
   * Aligns robot to the speaker, sets the shooter according to current distance from the speaker,
   * then feeds game piece when shooter is ready and heading is aligned.
   */
  public ShootNote() {
    var alignToSpeaker = AprilTagTrackDrive.getAlignToSpeaker();
    var autoSetShooter = new AutoSetShooter(SpinType.slowRightMotor);

    addCommands(
      new ParallelDeadlineGroup(
        new WaitForCounter(
          () -> alignToSpeaker.atTarget() && autoSetShooter.atTarget(),
          3,
          "Commands/ShootNote"
        ), // deadline
        alignToSpeaker,
        autoSetShooter
      ),
      RobotContainer.intake.getFeedNote()
    );
  }
}
