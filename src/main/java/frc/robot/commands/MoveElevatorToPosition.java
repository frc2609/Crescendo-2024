// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * Move elevator to a specific position.
 * If you want to move the elevator to a joystick position, just call Elevator.setTargetHeight().
 */
public class MoveElevatorToPosition extends Command {
  public static enum Position {
    intake(0.0),
    amp(0.92),
    trap(0.92);

    public final double targetHeight;    

    private Position(double targetHeight) {
      this.targetHeight = targetHeight;
    }
  }

  private final double targetHeight;

  /** Creates a new MoveElevatorToPosition. */
  public MoveElevatorToPosition(Position position) {
    this.targetHeight = position.targetHeight;
    addRequirements(RobotContainer.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // no need to log targetHeight here because Elevator will do it
    RobotContainer.elevator.setHeight(targetHeight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.elevator.atTarget();
  }
}
