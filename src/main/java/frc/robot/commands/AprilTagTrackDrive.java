// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTag.ID;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.DriveUtil;

/**
 * Drive the robot using translational velocity from the driver controller. Robot aligns heading to
 * the specified AprilTag.
 */
public class AprilTagTrackDrive extends Command {
  private final boolean isFieldRelative;
  private final ID blueAprilTagID;
  private final ID redAprilTagID;
  private ID trackedAprilTagID;
  private final Rotation2d rotationOffset;

  /**
   * Creates a new AprilTagTrackDrive.
   * @param isFieldRelative Whether or not to drive in field-relative mode.
   * @param blueAprilTagID The AprilTag ID to align the heading to when on the blue alliance.
   * @param redAprilTagID The AprilTag ID to align the heading to when on the red alliance.
   * @param rotationOffset An offset to apply to the apriltag's heading.
   */
  public AprilTagTrackDrive(boolean isFieldRelative, ID blueAprilTagID, ID redAprilTagID, Rotation2d rotationOffset) {
    addRequirements(RobotContainer.drive);
    this.isFieldRelative = isFieldRelative;
    this.blueAprilTagID = blueAprilTagID;
    this.redAprilTagID = redAprilTagID;
    this.rotationOffset = rotationOffset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.trackedAprilTagID = RobotContainer.isRedAlliance("AprilTagTrackDrive") ? redAprilTagID : blueAprilTagID;
    SmartDashboard.putNumber("AprilTagTrack/AprilTag ID", trackedAprilTagID.getID());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // strip the rotation component of the apriltag pose because we don't require it
    Pose2d apriltagPose = new Pose2d(Limelight.getTargetPose2d(trackedAprilTagID).getTranslation(), new Rotation2d());
    Transform2d relativePose = RobotContainer.drive.drive.getPose().minus(apriltagPose);
    Rotation2d heading =
      Rotation2d.fromRadians(Math.atan2(relativePose.getY(), relativePose.getX()))
      .plus(Rotation2d.fromDegrees(180)) // so the robot's front faces the apriltag
      .plus(rotationOffset);

    SmartDashboard.putNumber("AprilTagTrack/Target Heading (Deg)", heading.getDegrees());
    SmartDashboard.putNumber("AprilTagTrack/Current Heading (Deg)", RobotContainer.drive.drive.getYaw().getDegrees());

    double[] driverInputs = DriveUtil.getDriverInputs(
      RobotContainer.driverController,
      true,
      false,
      false,
      true,
      DriveUtil.getSensitivity(RobotContainer.driverController),
      isFieldRelative
    );

    ChassisSpeeds speeds = RobotContainer.drive.drive.swerveController.getTargetSpeeds(
      driverInputs[0],
      driverInputs[1],
      heading.getRadians(),
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
