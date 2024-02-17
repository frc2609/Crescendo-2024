// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.DriveUtil;

/**
 * TODO: Actual docs
 */
public class VisionTrackDrive extends Command {
  private final boolean isFieldRelative;
  private double tx;

  private Transform2d targetOffset;

  PIDController angularVelocityPID = new PIDController(0.05, 0.0, 0.0);

  /** Creates a new VisionTrackDrive. */
  public VisionTrackDrive(boolean isFieldRelative) {
    addRequirements(RobotContainer.drive);
    this.isFieldRelative = isFieldRelative;
    // TODO: add this to a table (e.g. vision/)
    SmartDashboard.putData("Angular Velocity PID", angularVelocityPID);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetOffset = RobotContainer.drive.drive.swerveDrivePoseEstimator.getEstimatedPosition().minus(Limelight.getTargetPose2d(Constants.AprilTag.ID.kRedSpeakerCenter));
    
    tx = Math.toDegrees(Math.atan(targetOffset.getY()/targetOffset.getX()));
    // reset saved state when the command starts; useful if 'i' term is used
    angularVelocityPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetOffset = RobotContainer.drive.drive.swerveDrivePoseEstimator.getEstimatedPosition().minus(Limelight.getTargetPose2d(Constants.AprilTag.ID.kRedSpeakerCenter));
    tx = Math.toDegrees(Math.atan(targetOffset.getY()/targetOffset.getX()));
    angularVelocityPID.setSetpoint(tx);
    double calculatedAngularVelocity = angularVelocityPID.calculate(RobotContainer.drive.drive.getPose().getRotation().getDegrees());

    // TODO: add this to a table (e.g. vision/)
    SmartDashboard.putNumber("Calculated Angular Velocity", calculatedAngularVelocity);
    SmartDashboard.putNumber("target tx", tx);
    SmartDashboard.putNumber("target offset x",targetOffset.getX());
    SmartDashboard.putNumber("target offset y", targetOffset.getY());
    SmartDashboard.putNumber("gyro angle", RobotContainer.drive.drive.getPose().getRotation().getDegrees());

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
