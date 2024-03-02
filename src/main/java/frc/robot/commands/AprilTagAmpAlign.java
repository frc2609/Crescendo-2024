// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTag.ID;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.DriveUtil;

public class AprilTagAmpAlign extends Command {

  private final boolean isFieldRelative;
  private final ID aprilTagID;

  PIDController xAlignController = new PIDController(0, 0, 0);

  /** Creates a new AmpAlign. */
  public AprilTagAmpAlign(boolean isFieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.isFieldRelative = isFieldRelative;
    this.aprilTagID = Constants.AprilTag.ID.kRedAmp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d targetOffset = RobotContainer.drive.drive.swerveDrivePoseEstimator.getEstimatedPosition().minus(Limelight.getTargetPose2d(aprilTagID));
    double tagHeading = Limelight.getTargetPose2d(aprilTagID).getRotation().getRadians();
    double[] driverInputs = DriveUtil.getDriverInputs(
      RobotContainer.driverController,
      true,
      false,
      false,
      true,
      DriveUtil.getSensitivity(RobotContainer.driverController)
    );

    ChassisSpeeds speeds = RobotContainer.drive.drive.swerveController.getTargetSpeeds(
      driverInputs[0],
      driverInputs[1],
      -tagHeading,
      RobotContainer.drive.drive.getPose().getRotation().getRadians(),
      RobotContainer.drive.getLimitedTeleopLinearSpeed()
    );

    if (isFieldRelative) {
      RobotContainer.drive.drive.driveFieldOriented(speeds);
    } else {
      RobotContainer.drive.drive.drive(speeds);
    }

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
