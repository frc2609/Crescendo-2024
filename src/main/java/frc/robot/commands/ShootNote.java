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
   * @param spinType Type of spin to put on the note.
   */
  public ShootNote(SpinType spinType) {
    var alignToSpeaker = new LineupWithSpeaker2D();
    var autoSetShooter = new AutoSetShooter2D(spinType);

    addCommands(
      new ParallelDeadlineGroup(
        new WaitForCounter(
          () -> alignToSpeaker.atTarget() && autoSetShooter.atTarget(),
          5,
          "Commands/ShootNote"
        ), // deadline
        alignToSpeaker,
        autoSetShooter,
        // run the intake if the note hasn't reached the sensor yet
        // this is necessary because this command group interrupts any intake commands
        RobotContainer.intake.getIntakeNote()//, 
        // TODO: temporarily commented out; makes auto work better but teleop becomes useless
        // RobotContainer.rearLimelight.getEstimateRobotPose(),
        // RobotContainer.sideLimelight.getEstimateRobotPose()
      ),
      RobotContainer.intake.getFeedNote(),
      new IdleShooter()
    );
  }
}
