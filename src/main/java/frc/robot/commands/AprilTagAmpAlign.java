// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTag.ID;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.DriveUtil;

/**
 * Face intake towards amp and align the robot to the amp with forward/backward velocity.
 * Left/right (towards/away from amp) velocity is controlled by the driver.
 * Automatically detects alliance colour.
 */
public class AprilTagAmpAlign extends Command {
  public static final double alignmentSpeedLimit = 2.0;
  private final PIDController xAlignController = new PIDController(0.5, 0, 0);
  private ID aprilTagID;

  /** Creates a new AprilTagAmpAlign. */
  public AprilTagAmpAlign() {
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // update alliance colour each time the command starts (e.g. if changed during testing)
    // also, if you do it in the constructor and the code starts before the DS or FMS connects, it will grab the default (wrong) alliance
    aprilTagID = RobotContainer.isRedAlliance("AprilTagAmpAlign") ? ID.kRedAmp : ID.kBlueAmp;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d tagPose = Limelight.getTargetPose2d(aprilTagID);
    Transform2d targetOffset = RobotContainer.drive.drive.swerveDrivePoseEstimator.getEstimatedPosition().minus(tagPose);
    double tagHeading = tagPose.getRotation().getRadians();

    double[] driverInputs = DriveUtil.getDriverInputs(
      RobotContainer.driverController,
      true,
      false,
      false,
      true,
      DriveUtil.getSensitivity(RobotContainer.driverController),
      true
    );

    double calculatedSpeed = MathUtil.clamp(xAlignController.calculate(targetOffset.getY(), 0), -alignmentSpeedLimit, alignmentSpeedLimit);
    SmartDashboard.putNumber("AprilTagAmpAlign/Alignment Speed (mps)", calculatedSpeed);

    ChassisSpeeds speeds = RobotContainer.drive.drive.swerveController.getTargetSpeeds(
      calculatedSpeed,
      driverInputs[1],
      -tagHeading,
      RobotContainer.drive.drive.getOdometryHeading().getRadians(),
      RobotContainer.drive.getLimitedTeleopLinearSpeed()
    );

    RobotContainer.drive.drive.driveFieldOriented(speeds);
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
