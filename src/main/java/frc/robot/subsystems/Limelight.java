// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.LimeLightHelpers;
import frc.robot.utils.LimeLightHelpers.LimelightTarget_Fiducial;
import frc.robot.utils.LimeLightHelpers.Results;

public class Limelight extends SubsystemBase {
  private final String limelightName;
  private Pose2d pose = new Pose2d();
  private boolean poseValid = false;
  private double latencyMS = 0;
  // reference: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
  private double xyStds = 0;
  private double degStds = 0;

  /**
   * Creates a new Limelight.
   * @param limelightName The limelight's name (in Limelight config!).
   */
  public Limelight(String limelightName) {
    this.limelightName = limelightName;
  }

  @Override
  public void periodic() {
    poseValid = false;

    if (LimeLightHelpers.getTV(limelightName)) {
      Results results = LimeLightHelpers.getLatestResults(limelightName).targetingResults;
      pose = results.getBotPose2d_wpiBlue();
      latencyMS = results.latency_capture / 1000.0;
      var odometryDifference = RobotContainer.drive.drive.getPose().minus(pose);
      var distance = Math.hypot(odometryDifference.getX(), odometryDifference.getY());
      var totalTargetArea = LimeLightHelpers.getTA(limelightName);

      // multiple targets detected
      if (results.targets_Fiducials.length >= 2) {
        xyStds = 0.5;
        degStds = 6;
        poseValid = true;
      }
      // 1 target with large area and close to estimated pose
      else if (totalTargetArea > 0.8 && distance < 0.5) {
        xyStds = 1.0;
        degStds = 12;
        poseValid = true;
      }
      // 1 target farther away and estimated pose is close
      else if (totalTargetArea > 0.1 && distance < 0.3) {
        xyStds = 2.0;
        degStds = 30;
        poseValid = true;
      }
      // conditions don't match to add a vision measurement
      else {
        xyStds = 0;
        degStds = 0;
        poseValid = false;
      }

      // only run when pose valid
      SmartDashboard.putNumber("Limelight/" + limelightName + "/Latency (MS)", latencyMS);
      SmartDashboard.putNumber("Limelight/" + limelightName + "/Odometry Error", distance);
      SmartDashboard.putNumber("Limelight/" + limelightName + "/Best Target Area", getBestTargetArea(results.targets_Fiducials));
      SmartDashboard.putNumber("Limelight/" + limelightName + "/Total Target Area", totalTargetArea);
    }

    // always run
    SmartDashboard.putBoolean("Limelight/" + limelightName + "/Pose Valid", poseValid);
    RobotContainer.drive.drive.field.getObject(limelightName + " Estimated Pose").setPose(pose);
  }

  public void resetOdometryToVision() {
    if (poseValid) {
      RobotContainer.drive.drive.resetOdometry(pose);
    }
  }

  public void addPoseEstimate(boolean setGyro) {
    var poseToSet = setGyro ? pose : new Pose2d(pose.getTranslation(), new Rotation2d());

    if (poseValid && RobotContainer.drive.getVelocity() < 0.05 && RobotContainer.drive.drive.getRobotVelocity().omegaRadiansPerSecond < Math.toRadians(10)) {
      RobotContainer.drive.drive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds))); 
      RobotContainer.drive.drive.swerveDrivePoseEstimator.addVisionMeasurement(poseToSet, Timer.getFPGATimestamp() - latencyMS);
    }
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
}
