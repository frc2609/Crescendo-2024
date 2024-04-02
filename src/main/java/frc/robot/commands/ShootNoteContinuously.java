// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ShooterFlywheel.SpinType;


public class ShootNoteContinuously extends ParallelCommandGroup {
  /**
   * Aligns robot to the speaker while setting the shooter according to current distance from the
   * speaker. Runs until cancelled.
   */
  public ShootNoteContinuously() {
    addCommands(
      new LineupWithSpeaker2D(),
      new AutoSetShooter2D(SpinType.slowRightMotor)
    );
  }
}
