// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFlywheel.SpinType;

public class IntakeAndShootContinously extends SequentialCommandGroup {
  /**
   * While aligning robot to speaker and automatically setting shooter, run intake until note is
   * detected, then shoot note when ready. Once note is scored, intake + shoot sequence repeats.
   */
  public IntakeAndShootContinously() {
    var alignToSpeaker = AprilTagTrackDrive.getAlignToSpeaker();
    var autoSetShooter = new AutoSetShooter(SpinType.slowRightMotor);

    addCommands(
      new ParallelCommandGroup(
        alignToSpeaker,
        autoSetShooter,
        new SequentialCommandGroup(
          RobotContainer.intake.getIntakeNote(),
          new WaitForCounter(
            () -> alignToSpeaker.atTarget() && autoSetShooter.atTarget(),
            5,
            "Commands/IntakeAndShootNote"
          ),
          RobotContainer.intake.getFeedNote()
        ).repeatedly()
      ),
      new IdleShooter()
    );
  }
}
