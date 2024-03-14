// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.MoveElevatorToPosition.Position;

public class AutoScoreAmp extends SequentialCommandGroup {
  /**
   * Automatically score a note in the amp.
   * <p> Aligns to the amp, raises elevator when close to the amp, scores game piece once aligned,
   * then resets elevator.
   * <p> Does not lower elevator if interrupted.
   * @param alignmentSpeedAxis A multiplier from 0-1 to apply to the translation speed while driving to the amp.
   */
  public AutoScoreAmp(Supplier<Double> alignmentSpeedAxis) {
    var alignToAmp = new AlignToAmp(alignmentSpeedAxis);

    addCommands(
      new ParallelCommandGroup(
        alignToAmp,
        new SequentialCommandGroup(
          new WaitForCounter(() -> alignToAmp.getEstimatedTimeToTarget() < 0.5 || Math.abs(alignToAmp.getTranslationError()) < 1.5, 5, "Commands/AutoScoreAmp"),
          new MoveElevatorToPosition(Position.amp)
        )
      ),
      RobotContainer.intake.getExpelNote(),
      Commands.waitSeconds(0.25),
      new ResetIntakeAndElevator()
    );
  }
}
