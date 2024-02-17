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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.LimeLightHelpers;
import frc.robot.utils.LimeLightHelpers.LimelightResults;
import frc.robot.utils.LimeLightHelpers.LimelightTarget_Fiducial;
import frc.robot.utils.LimeLightHelpers.Results;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  public static Pose2d limelightPose = new Pose2d();
  public static Field2d fieldPose = new Field2d();

  public Limelight() {}

  @Override
  public void periodic() {

    Results results = LimeLightHelpers.getLatestResults("limelight").targetingResults;
    
    if(results.valid) {
      
      limelightPose = results.getBotPose2d_wpiBlue();
      fieldPose.setRobotPose(limelightPose);
      
      Transform2d odometryDifference = RobotContainer.drive.drive.field.getRobotPose().minus(limelightPose);
      double distance = Math.sqrt(Math.pow(odometryDifference.getX(), 2) + Math.pow(odometryDifference.getY(), 2));
      SmartDashboard.putNumber("odometry error", distance);

      double xyStds;
      double degStds;
      
      int numTargets = results.targets_Fiducials.length;

      if (numTargets >= 2) {
        xyStds = 0.5;
        degStds = 6;
      } else if (getBestTargetArea(results.targets_Fiducials) > 0.8 && distance < 0.5) {
        xyStds = 1.0;
        degStds = 12;
      } else if (getBestTargetArea(results.targets_Fiducials) > 0.1 && distance < 0.3) {
        xyStds = 2.0;
        degStds = 30;
      } else {
        // WARNING: Anything below this will not get executed if conditions don't match
        return;
      }

      RobotContainer.drive.drive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
      
      if (distance > 0.1) {
        RobotContainer.drive.drive.swerveDrivePoseEstimator.addVisionMeasurement(limelightPose, Timer.getFPGATimestamp() - (results.latency_capture/1000.0));
      }
    }

    Optional<Translation3d> acceleration = RobotContainer.drive.drive.getAccel();

    if (acceleration.isPresent()) {
      SmartDashboard.putNumber("drive accel", acceleration.get().getNorm());
    }

    SmartDashboard.putData("limelightFieldPose", fieldPose);

  }

  public static double getBestTargetArea(LimelightTarget_Fiducial[] targets) {
    double largestArea = 0;
    for (LimelightTarget_Fiducial target : targets) {
      if (target.ta > largestArea){
        largestArea = target.ta;
      }
    }
    return largestArea;
  }

  public static Pose2d getTargetPose2d(Constants.AprilTag.ID targetID) {
    Optional<Pose3d> targetPose3d = Constants.AprilTag.fieldLayout.getTagPose(targetID.getID());
    Pose2d targetPose2d = new Pose2d();
    if (targetPose3d.isPresent()) {
      Rotation2d targetRotation = new Rotation2d(targetPose3d.get().getRotation().getX(), targetPose3d.get().getRotation().getY());
      targetPose2d = new Pose2d(targetPose3d.get().getX(), targetPose3d.get().getY(), targetRotation);
    }

    return targetPose2d;
  }
}
