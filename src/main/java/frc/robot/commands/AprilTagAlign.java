// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTag.ID;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;

public class AprilTagAlign extends Command {

  private final boolean isFieldRelative;
  private final ID aprilTagID;

  PIDController xAlignController = new PIDController(0, 0, 0);

  /** Creates a new AmpAlign. */
  public AprilTagAlign(boolean isFieldRelative, ID aprilTagID) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.isFieldRelative = isFieldRelative;
    this.aprilTagID = aprilTagID;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d targetOffset = RobotContainer.drive.drive.swerveDrivePoseEstimator.getEstimatedPosition().minus(Limelight.getTargetPose2d(aprilTagID));
    double tx = Math.atan(targetOffset.getY() / targetOffset.getX());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
