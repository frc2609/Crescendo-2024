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
  private final ID aprilTagID;

  /**
   * Creates a new AprilTagTrackDrive.
   * @param isFieldRelative Whether or not to drive in field-relative mode.
   * @param aprilTagID The AprilTag ID to align the heading to.
   */
  public AprilTagTrackDrive(boolean isFieldRelative, ID aprilTagID) {
    addRequirements(RobotContainer.drive);
    this.isFieldRelative = isFieldRelative;
    this.aprilTagID = aprilTagID;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("AprilTagTrack/AprilTag ID", aprilTagID.getID());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d targetOffset = RobotContainer.drive.drive.swerveDrivePoseEstimator.getEstimatedPosition().minus(Limelight.getTargetPose2d(aprilTagID));
    double tx = Math.atan(targetOffset.getY() / targetOffset.getX());
    
    SmartDashboard.putNumber("AprilTagTrack/Target tx", tx);
    SmartDashboard.putNumber("AprilTagTrack/Target Offset X", targetOffset.getX());
    SmartDashboard.putNumber("AprilTagTrack/Target Offset Y", targetOffset.getY());
    SmartDashboard.putNumber("AprilTagTrack/Current Heading (Deg)", RobotContainer.drive.drive.getYaw().getDegrees());

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
      tx,
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
