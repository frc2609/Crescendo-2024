// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.LimeLightHelpers;
import frc.robot.utils.LimeLightHelpers.LimelightTarget_Fiducial;
import frc.robot.utils.LimeLightHelpers.Results;

public class Limelight extends SubsystemBase {
  public static Pose2d limelightPose = new Pose2d();
  public static double numTargets_dt = -1; // time it takes to calculate how many targets we see

  /** Creates a new Limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    // Only consider updating odometry if the reading from limelight is valid
    // False when no valid tags detected
    boolean isValid = LimeLightHelpers.getTV("limelight-shooter");
    if (isValid) {
      // Results results = LimeLightHelpers.getLatestResults("limelight").targetingResults;

      // Record recently detected pose
      // limelightPose = results.getBotPose2d_wpiBlue();
      limelightPose = LimeLightHelpers.getBotPose2d_wpiBlue("limelight-shooter");

      // Calculate the vector between the current drivetrain pose and vision pose
      Transform2d odometryDifference = RobotContainer.drive.drive.field.getRobotPose().minus(limelightPose);
      // Calculate the cartesian distance between poses
      double distance = Math.sqrt(Math.pow(odometryDifference.getX(), 2) + Math.pow(odometryDifference.getY(), 2));
      // Record odometry error to smartdashboard
      SmartDashboard.putNumber("swerve/Odometry Error", distance);

      // reference: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization
      double xyStds;
      double degStds;
      
      // int numTargets = results.targets_Fiducials.length;
      int numTargets = Limelight.getNumTargetsFast("limelight-shooter");
      SmartDashboard.putNumber("Target area", LimeLightHelpers.getTA("limelight-shooter"));
      double area = LimeLightHelpers.getTA("limelight-shooter");
      if (numTargets >= 2) {
        // trust vision odometry more if we see more than one apriltag
        // orientation should also be more accurate in this case
        if(area < 0.4 && area > 0.3){
          xyStds = 1;
          degStds = 6;
        }else if(area <= 0.3){
          xyStds = 1.5;
          degStds = 6;
        }
        else{
          xyStds = 0.5;
          degStds = 6;
        }
      } else if (area> 0.8 && distance < 0.5) {
        xyStds = 1.0;
        degStds = 12;
      } else if (area > 0.1 && distance < 0.3) {
        xyStds = 2.0;
        degStds = 30;
      } else {
        // WARNING: Anything below this will not get executed if conditions don't match
        return;
      }

      // crash the code so I can see why it went wrong (makes it nice and detectable)
      if (!isValid) throw new RuntimeException("Limelight results invalid after validity check!");

      // set the "trust factor" of the vision measurement
      RobotContainer.drive.drive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
      
      if (distance > 0.02 && RobotContainer.drive.getVelocity() < 0.05) {
        // add a vision measurement if the cartesian error in odometry is greater than 0.02m
        RobotContainer.drive.drive.swerveDrivePoseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() - (LimeLightHelpers.getLatency_Capture("limelight-shooter")/1000.0 - (LimeLightHelpers.getLatency_Pipeline("limelight-shooter")/1000.0)));
      }
    }

    RobotContainer.drive.drive.field.getObject("Limelight Estimated Pose").setPose(limelightPose);
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
      targetPose2d = targetPose3d.get().toPose2d();
    }
    return targetPose2d;
  }
  public static int getNumTargetsFast(String limelightName){
    String jsonDump = LimeLightHelpers.getJSONDump(limelightName);
    double start = Timer.getFPGATimestamp();
    Pattern pattern = Pattern.compile("\"fID\":\\d+");
    Matcher matcher = pattern.matcher(jsonDump);

    int count = 0;
    while (matcher.find()){
      count++;
    }
    numTargets_dt = Timer.getFPGATimestamp()-start;
    return count;
  }
}
