// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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

  /**
   * Creates a new AprilTagTrackDrive.
   * @param isFieldRelative Whether or not to drive in field-relative mode.
   * @param blueAprilTagID The AprilTag ID to align the heading to when on the blue alliance.
   * @param redAprilTagID The AprilTag ID to align the heading to when on the red alliance.
   */
  public AprilTagTrackDrive(boolean isFieldRelative, ID blueAprilTagID, ID redAprilTagID) {
    addRequirements(RobotContainer.drive);
    this.isFieldRelative = isFieldRelative;
    this.blueAprilTagID = blueAprilTagID;
    this.redAprilTagID = redAprilTagID;
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
    Transform2d targetOffset = RobotContainer.drive.drive.getPose().minus(Limelight.getTargetPose2d(trackedAprilTagID));
    // TODO: Make functional for stage
    // TODO: Less Magic
    double heading = Math.atan(targetOffset.getY() / targetOffset.getX()) - Limelight.getTargetPose2d(trackedAprilTagID).getRotation().getRadians() - Math.PI*Math.cos(Limelight.getTargetPose2d(trackedAprilTagID).getRotation().getRadians());

    SmartDashboard.putNumber("AprilTagTrack/Target Heading (Deg)", Math.toDegrees(heading));
    SmartDashboard.putNumber("AprilTagTrack/Target Offset X", targetOffset.getX());
    SmartDashboard.putNumber("AprilTagTrack/Target Offset Y", targetOffset.getY());
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
      heading,
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
