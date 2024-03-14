// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.utils.LimeLightHelpers;
import frc.robot.utils.LimeLightHelpers.Results;

public class Limelight extends SubsystemBase {
  public static enum Pipeline {
    localizeRobot(0),
    trackNote(1);

    public int id;

    private Pipeline(int id) {
      this.id = id;
    }
  }

  private final String name;

  /** Creates a new Limelight. */
  public Limelight(String name) {
    this.name = name;
  }

  @Override
  public void periodic() {}

  public RunCommand getEstimateRobotPose(boolean setGyro) {
    setPipeline(Pipeline.localizeRobot);
    return new RunCommand(() -> {
      if (!LimeLightHelpers.getTV(name)) { 
        SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", false);
        return;
      }
      Results results = LimeLightHelpers.getLatestResults(name).targetingResults;
      var pose = results.getBotPose2d_wpiBlue();
      var latencyMS = results.latency_capture / 1000.0;
      var odometryDifference = RobotContainer.drive.drive.getPose().minus(pose);
      var distance = Math.hypot(odometryDifference.getX(), odometryDifference.getY());
      var totalTargetArea = LimeLightHelpers.getTA(name);

      double xyStds;
      double degStds;

      // multiple targets detected
      if (results.targets_Fiducials.length >= 2) {
        xyStds = 0.5;
        degStds = 6;
      }
      // 1 target with large area and close to estimated pose
      else if (totalTargetArea > 0.8 && distance < 0.5) {
        xyStds = 1.0;
        degStds = 12;
      }
      // 1 target farther away and estimated pose is close
      else if (totalTargetArea > 0.1 && distance < 0.3) {
        xyStds = 2.0;
        degStds = 30;
      }
      // conditions don't match to add a vision measurement
      else {
        xyStds = 0;
        degStds = 0;
        return;
      }

      SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", true);
      SmartDashboard.putNumber("Limelight/" + name + "/Latency (MS)", latencyMS);
      SmartDashboard.putNumber("Limelight/" + name + "/Odometry Error", distance);
      SmartDashboard.putNumber("Limelight/" + name + "/Total Target Area", totalTargetArea);
      RobotContainer.drive.drive.field.getObject(name + " Estimated Pose").setPose(pose);
    
      var poseToSet = setGyro ? pose : new Pose2d(pose.getTranslation(), new Rotation2d());

      // check if robot is moving slowly first
      if (RobotContainer.drive.getVelocity() < 0.05 && RobotContainer.drive.drive.getRobotVelocity().omegaRadiansPerSecond < Math.toRadians(10)) {
        RobotContainer.drive.drive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds))); 
        RobotContainer.drive.drive.swerveDrivePoseEstimator.addVisionMeasurement(poseToSet, Timer.getFPGATimestamp() - latencyMS);
      }
    }, this);
  }

  public InstantCommand getResetRobotPose() {
    setPipeline(Pipeline.localizeRobot);
    return new InstantCommand(() -> {
      if (!LimeLightHelpers.getTV(name)) { 
        SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", false);
        return;
      }
      SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", true);
      Results results = LimeLightHelpers.getLatestResults(name).targetingResults;
      var pose = results.getBotPose2d_wpiBlue();

      RobotContainer.drive.drive.field.getObject(name + " Estimated Pose").setPose(pose);
      RobotContainer.drive.drive.resetOdometry(pose);
    }, this);
  }

  public void setPipeline(Pipeline pipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline.id);
  }
}
