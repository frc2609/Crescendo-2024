// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTag;
import frc.robot.utils.LimeLightHelpers;

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
  public static double numTargets_dt = 0.0;

  /** Creates a new Limelight. */
  public Limelight(String name) {
    this.name = name;
    SmartDashboard.putNumber(name + " distance Std Devs", 0.1);
    SmartDashboard.putNumber(name + " velocity Std Devs", 0.5);
  }

  @Override
  public void periodic() {}

  public RunCommand getEstimateRobotPose(boolean setGyro) {
    setPipeline(Pipeline.localizeRobot); // oh that's horrible btw: this gets called when the command is created, not when it actually runs (as it should)
    return new RunCommand(this::updateOdometry, this);
  }

  public InstantCommand getResetRobotPose() {
    setPipeline(Pipeline.localizeRobot); // same issue here!
    return new InstantCommand(() -> {
      if (!LimeLightHelpers.getTV(name)) { 
        SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", false);
        return;
      }
      SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", true);
      var pose = LimeLightHelpers.getBotPose2d_wpiBlue(name);

      RobotContainer.drive.drive.field.getObject(name + " Estimated Pose").setPose(pose);
      RobotContainer.drive.drive.resetOdometry(pose);
    }, this);
  }

  public void setPipeline(Pipeline pipeline) {
    NetworkTableInstance.getDefault().getTable(name).getEntry("pipeline").setNumber(pipeline.id);
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

  // check in periodic and set to member variable???
  public Optional<Pose2d> getPose() {
    if (!LimeLightHelpers.getTV(name)) { 
      SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", false);
      return Optional.empty();
    } else {
      var pose = LimeLightHelpers.getBotPose2d_wpiBlue(name);
      SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", true);
      RobotContainer.drive.drive.field.getObject(name + " Estimated Pose").setPose(pose);
      return Optional.of(pose);
    }
  }

  public void updateOdometry() {
    var detectedPose = getPose();
    if (!isPoseValid(detectedPose)) return;
    if (!isPoseValid(RobotContainer.drive.drive.getPose()) && movingSlowly()) {
      RobotContainer.drive.drive.resetOdometry(detectedPose.get());
    } else {
      double latencyMS = LimeLightHelpers.getLatency_Capture(name) / 1000.0;
      double[] stdDevs = lazyStdDevs();

      RobotContainer.drive.drive.addVisionMeasurement(detectedPose.get(), Timer.getFPGATimestamp() - latencyMS, VecBuilder.fill(stdDevs[0], stdDevs[0], Units.degreesToRadians(stdDevs[1])));
      
      SmartDashboard.putNumber("Limelight/" + name + "/Latency (MS)", latencyMS);
      SmartDashboard.putNumber("Limelight/" + name + "/Odometry Error", getOdometryDifference(detectedPose.get()));
      SmartDashboard.putNumber("Limelight/" + name + "/Total Target Area", LimeLightHelpers.getTA(name));
    }
  }

  public double[] lazyStdDevs() {
    double distanceStdDevMultiplier = SmartDashboard.getNumber(name + " distance Std Devs", 0.1);
    double velocityStdDevMultiplier = SmartDashboard.getNumber(name + " velocity Std Devs", 0.5);
    double tagArea = LimeLightHelpers.getTA(name); // SKETCHY
    double xyStds = distanceStdDevMultiplier * (1 / tagArea) + velocityStdDevMultiplier * RobotContainer.drive.getVelocity();
    double rotStds = 2;
    SmartDashboard.putNumber("TA", tagArea);
    SmartDashboard.putNumber("XY STDS", xyStds);
    SmartDashboard.putNumber("ROT STDS", rotStds);
    return new double[] { xyStds, rotStds };
  }

  /**
   * Calculate the distance between the current vision and odometry poses.
   * @param pose Estimated Vision Pose
   * @return Linear distance between vision and odometry poses.
   */
  public static double getOdometryDifference(Pose2d pose) {
    var odometryDifference = RobotContainer.drive.drive.getPose().minus(pose);
    return Math.hypot(odometryDifference.getX(), odometryDifference.getY());
  }

  /**
   * Check whether pose is within game field.
   * @param pose Pose to check.
   * @return Whether pose is within game field.
   */
  public static boolean isPoseValid(Pose2d pose) {
    return (pose.getX() >= 0 && pose.getX() <= AprilTag.fieldLayout.getFieldLength())
      && (pose.getY() >= 0 && pose.getY() <= AprilTag.fieldLayout.getFieldWidth());
  }

  /**
   * Check whether pose is present and within game field.
   * @param pose Optional<Pose2d> to check.
   * @return Whether pose is present and within game field.
   */
  public static boolean isPoseValid(Optional<Pose2d> pose) {
    if (pose.isPresent()) {
      return isPoseValid(pose.get());
    } else {
      return false;
    }
  }

  /**
   * Check whether the robot is translating and rotating slowly.
   * @return Whether the robot is translating and rotating slowly.
   */
  public static boolean movingSlowly() {
    boolean rotationSlow = RobotContainer.drive.drive.getRobotVelocity().omegaRadiansPerSecond < Math.toDegrees(60);
    boolean driveSlow = RobotContainer.drive.getVelocity() < 1.0;
    return rotationSlow && driveSlow;
  }
}
