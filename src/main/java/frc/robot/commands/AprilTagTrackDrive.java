// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTag;
import frc.robot.Constants.AprilTag.ID;

/**
 * Align robot heading to AprilTag according to automatically-detected alliance colour.
 * Works standalone or alongside a command controlling robot translation (or PathPlanner).
 */
public class AprilTagTrackDrive extends Command {
  private final ID blueAprilTagID;
  private final ID redAprilTagID;
  private ID trackedAprilTagID;
  private final Rotation2d headingOffset;
  private Timer odometryValidTimer = new Timer();
  private boolean aligningToTag = false;
  // exists since 'isScheduled()' doesn't work when command is scheduled as part of a group
  private boolean isRunning = false;

  /**
   * Creates a new AprilTagTrackDrive.
   * This class has premade configurations (e.g. {@code AprilTagTrackDrive.getAlignToSpeaker()}),
   * so consider using those before using this constructor.
   * @param blueAprilTagID The AprilTag ID to align the heading to when on the blue alliance.
   * @param redAprilTagID The AprilTag ID to align the heading to when on the red alliance.
   * @param headingOffset Applies an offset to the robot's heading (e.g. to face tag with shooter instead of intake).
   */
  public AprilTagTrackDrive(ID blueAprilTagID, ID redAprilTagID, Rotation2d headingOffset) {
    this.blueAprilTagID = blueAprilTagID;
    this.redAprilTagID = redAprilTagID;
    this.headingOffset = headingOffset;
    // does not require drive since this is intended to be used in addition to a command controlling translation
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    odometryValidTimer.restart();
    trackedAprilTagID = RobotContainer.isRedAlliance("AprilTagTrackDrive") ? redAprilTagID : blueAprilTagID;
    SmartDashboard.putNumber("AprilTagTrack/AprilTag ID", trackedAprilTagID.getID());
    isRunning = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d aprilTagPose = AprilTag.getPose2d(trackedAprilTagID);
    Rotation2d heading;

    if (!RobotContainer.drive.odometryOutOfRange() && (RobotContainer.rearLimelight.isPoseValid() || RobotContainer.sideLimelight.isPoseValid())) {
      odometryValidTimer.restart();
      heading = getHeadingToTag();
      aligningToTag = true;
    } else if (!odometryValidTimer.hasElapsed(0.5)) {
      // if we lose heading, wait for a bit before reverting to gyro
      heading = getHeadingToTag();
      aligningToTag = true;
    } else {
      // only align to tag heading because odometry isn't reliable
      heading = aprilTagPose.getRotation();
      aligningToTag = false;
    }

    heading.plus(Rotation2d.fromDegrees(180)) // so the robot's front faces the apriltag
      .plus(headingOffset);
    
    SmartDashboard.putNumber("AprilTagTrack/Target Heading (Deg)", heading.getDegrees());
    SmartDashboard.putBoolean("AprilTagTrack/Aligning to Odometry", aligningToTag);
    SmartDashboard.putBoolean("AprilTagTrack/At Target", atTarget());

    RobotContainer.drive.overrideHeading(heading);
  }

  @Override
  public void end(boolean interrupted) {
    odometryValidTimer.stop();
    isRunning = false;
  }

  public boolean atTarget() {
    return isRunning && aligningToTag && RobotContainer.drive.drive.swerveController.thetaController.atSetpoint();
  }

  private Rotation2d getHeadingToTag() {
    // strip the rotation component of the apriltag pose so coordinates align with the field
    Pose2d aprilTagPose = new Pose2d(AprilTag.getPose2d(trackedAprilTagID).getTranslation(), new Rotation2d());
    Transform2d relativePose = RobotContainer.drive.drive.getPose().minus(aprilTagPose);
    return Rotation2d.fromRadians(Math.atan2(relativePose.getY(), relativePose.getX()));
  }

  // --- Common Configurations ---

  public static AprilTagTrackDrive getAlignToSpeaker() {
    return new AprilTagTrackDrive(ID.kBlueSpeakerCenter, ID.kRedSpeakerCenter, Rotation2d.fromDegrees(180));
  }
}
