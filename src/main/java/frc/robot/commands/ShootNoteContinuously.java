// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFlywheel.SpinType;


public class ShootNoteContinuously extends ParallelCommandGroup {
  /**
   * Aligns robot to the speaker while setting the shooter according to current distance from the
   * speaker. While there is no note, runs the intake. When there is a note, attempts to shoot it.
   * Runs until cancelled.
   */
  public ShootNoteContinuously() {
    var alignToSpeaker = new LineupWithSpeaker2D();
    var autoSetShooter = new AutoSetShooter2D(SpinType.slowRightMotor);

    addCommands(
      alignToSpeaker,
      autoSetShooter,
      new RepeatCommand(
        new SequentialCommandGroup(
          RobotContainer.intake.getIntakeNote(),
          new WaitForCounter(
            () -> alignToSpeaker.atTarget() && autoSetShooter.atTarget(),
            5,
            "Commands/ShootNote"
          ), // deadline
          RobotContainer.intake.getFeedNote()
        )
      )
    );
  }
}
