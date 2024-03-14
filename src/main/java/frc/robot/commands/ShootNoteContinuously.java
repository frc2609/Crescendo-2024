// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFlywheel.SpinType;


public class ShootNoteContinuously extends ParallelCommandGroup {
  /**
   * Aligns robot to the speaker, sets the shooter according to current distance from the speaker,
   * Runs until cancelled.
   */
  public ShootNoteContinuously() {
    addCommands(
      AprilTagTrackDrive.getAlignToSpeaker(),
      new AutoSetShooter(SpinType.slowRightMotor),
      new InstantCommand(() -> RobotContainer.intake.setMotor(0.7), RobotContainer.intake)
    );
  }
}
