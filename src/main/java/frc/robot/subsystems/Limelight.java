// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.LimeLightHelpers;
import frc.robot.utils.LimeLightHelpers.LimelightTarget_Fiducial;
import frc.robot.utils.LimeLightHelpers.Results;

public class Limelight extends SubsystemBase {
  public static Pose2d limelightPose = new Pose2d();
  // use YAGSL's, and add the estimated pose under a different entry.
  public static Field2d fieldPose = new Field2d();

  /** Creates a new Limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    Results results = LimeLightHelpers.getLatestResults("limelight").targetingResults;

    // Only consider updating odometry if the reading from limelight is valid
    // TODO: investigate exactly under what conditions this fails
    if (results.valid) {
      // Record recently detected pose
      limelightPose = results.getBotPose2d_wpiBlue();
      fieldPose.setRobotPose(limelightPose);

      // Calculate the vector between the current drivetrain pose and vision pose
      Transform2d odometryDifference = RobotContainer.drive.drive.field.getRobotPose().minus(limelightPose);
      // Calculate the cartesian distance between poses
      double distance = Math.sqrt(Math.pow(odometryDifference.getX(), 2) + Math.pow(odometryDifference.getY(), 2));
      // Record odometry error to smartdashboard
      // TODO: Add this to beaverlogger instead?
      SmartDashboard.putNumber("odometry error", distance);

      // reference: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
      double xyStds;
      double degStds;
      
      int numTargets = results.targets_Fiducials.length;
      
      if (numTargets >= 2) {
        // trust vision odometry more if we see more than one apriltag
        // orientation should also be more accurate in this case
        xyStds = 0.5;
        degStds = 6;
      } else if (getBestTargetArea(results.targets_Fiducials) > 0.8 && distance < 0.5) {
        // TODO: check the range of the target area returned by getBestTargetArea()
        xyStds = 1.0;
        degStds = 12;
      } else if (getBestTargetArea(results.targets_Fiducials) > 0.1 && distance < 0.3) {
        xyStds = 2.0;
        degStds = 30;
      } else {
        // WARNING: Anything below this will not get executed if conditions don't match
        return;
      }

      // set the "trust factor" of the vision measurement
      RobotContainer.drive.drive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
      
      if (distance > 0.1) {
        // add a vision measurement if the cartesian error in odometry is greater than 0.1m
        // TODO, also subtract results.latency_pipeline/1000.0
        // TODO: line 79 (vision measurement std devs) can be set in this function
        RobotContainer.drive.drive.swerveDrivePoseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() - (results.latency_capture/1000.0));
      }
    }

    // TODO: this should not be in periodic. You only need to call it once
    SmartDashboard.putData("limelightFieldPose", fieldPose);
  }

  /**
   * Helper function to return the area of the largest target used for
   * determining how accurate our pose estimation is.
   * @param targets the list of detected targets
   * @return the area of the largest target
   */
  public static double getBestTargetArea(LimelightTarget_Fiducial[] targets) {
    double largestArea = 0;
    for (LimelightTarget_Fiducial target : targets) {
      if (target.ta > largestArea){
        largestArea = target.ta;
      }
    }
    return largestArea;
  }

  /**
   * Helper function that returns the 2d pose of the requested AprilTag ID
   * @param targetID enum representing the desired target
   * @return Pose2d of the requested target
   */
  public static Pose2d getTargetPose2d(Constants.AprilTag.ID targetID) {
    Optional<Pose3d> targetPose3d = Constants.AprilTag.fieldLayout.getTagPose(targetID.getID());
    Pose2d targetPose2d = new Pose2d(); // NOTE: if targetPose3d is NOT present, we will just return this
    if (targetPose3d.isPresent()) {
      // Convert to 2D
      Rotation2d targetRotation = new Rotation2d(targetPose3d.get().getRotation().getX(), targetPose3d.get().getRotation().getY());
      targetPose2d = new Pose2d(targetPose3d.get().getX(), targetPose3d.get().getY(), targetRotation);
    }
    return targetPose2d;
  }
}
