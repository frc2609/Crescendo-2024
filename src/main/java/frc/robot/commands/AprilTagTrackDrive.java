// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private PIDController angularVelocityPID = new PIDController(0.05, 0.0, 0.0);

  /**
   * Creates a new AprilTagTrackDrive.
   * @param isFieldRelative Whether or not to drive in field-relative mode.
   * @param aprilTagID The AprilTag ID to align the heading to.
   */
  public AprilTagTrackDrive(boolean isFieldRelative, ID aprilTagID) {
    addRequirements(RobotContainer.drive);
    this.isFieldRelative = isFieldRelative;
    this.aprilTagID = aprilTagID;
    SmartDashboard.putData("AprilTagTrack/Angular Velocity PID", angularVelocityPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("AprilTagTrack/AprilTag ID", aprilTagID.getID());
    // reset saved state when the command starts; useful if 'i' term is used
    angularVelocityPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d targetOffset = RobotContainer.drive.drive.swerveDrivePoseEstimator.getEstimatedPosition().minus(Limelight.getTargetPose2d(aprilTagID));
    double tx = Math.toDegrees(Math.atan(targetOffset.getY()/targetOffset.getX()));
    angularVelocityPID.setSetpoint(tx); // TODO: this can be given as a second parameter in 'calculate'
    double calculatedAngularVelocity = angularVelocityPID.calculate(RobotContainer.drive.drive.getPose().getRotation().getDegrees());

    SmartDashboard.putNumber("AprilTagTrack/Target tx", tx);
    SmartDashboard.putNumber("AprilTagTrack/Target Offset X", targetOffset.getX());
    SmartDashboard.putNumber("AprilTagTrack/Target Offset Y", targetOffset.getY());
    SmartDashboard.putNumber("AprilTagTrack/Current Heading (Deg)", RobotContainer.drive.drive.getYaw().getDegrees());
    SmartDashboard.putNumber("AprilTagTrack/Calculated Angular Velocity", calculatedAngularVelocity);

    double[] driverInputs = DriveUtil.getDriverInputs(
      RobotContainer.driverController,
      true,
      false,
      false,
      true,
      DriveUtil.getSensitivity(RobotContainer.driverController)
    );

    RobotContainer.drive.drive.drive(
      new Translation2d(driverInputs[0], driverInputs[1])
        .times(RobotContainer.drive.getLimitedTeleopLinearSpeed()),
      calculatedAngularVelocity,
      isFieldRelative,
      false
    );
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
