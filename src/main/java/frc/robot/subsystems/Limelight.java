// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTag;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.TunableNumber;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

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
  private final TunableNumber velocityStdDevMultiplier;
  private final TunableNumber distanceStdDevMultiplier;

  /**
   * Creates a new Limelight.
   * @param name Limelight NetworkTables name.
   */
  public Limelight(String name) {
    this.name = name;
    distanceStdDevMultiplier = new TunableNumber("Limelight/" + name + "/Distance Std Dev Multiplier", 0.1);
    velocityStdDevMultiplier = new TunableNumber("Limelight/" + name + "/Velocity Std Dev Multiplier", 0.5);

    // create the limelight pose object immediately instead of the first time it is used
    RobotContainer.drive.drive.field.getObject(name + " Estimated Pose");

    // setup port forwarding
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, name + ".local", port);
    }
  }

  @Override
  public void periodic() {}

  // TODO: try all of these (or a combination of these) to see which is most effective

  // why: accuracy decreases with tag distance and velocity
  public double calculateXYStdDevs1(PoseEstimate measurement) {
    return measurement.avgTagDist * distanceStdDevMultiplier.get()
      + Math.max(Math.abs(RobotContainer.drive.getVelocity()) * velocityStdDevMultiplier.get(), 0);
  }

  // why: accuracy increases substantially with more than 1 tag
  public double calculateXYStdDevs2(PoseEstimate measurement) {
    return measurement.tagCount >= 2 ? 0.7 : 1.4;
  }

  // why: accuracy increases substantially with more than 1 tag (more aggressive version of #2)
  public double calculateXYStdDevs3(PoseEstimate measurement) {
    return measurement.tagCount >= 2 ? 0.7 : 9999999;
  }

  // why: invalid poses often have inaccurate rotation components
  public double calculateXYStdDevs4(PoseEstimate measurement) {
    double rotationError = RobotContainer.drive.getPoseEfficiently().getRotation()
      .minus(measurement.pose.getRotation()).getDegrees();
    return (rotationError / 45.0); // 1m std dev per ___ deg off
  }

  /**
   * Add the current detected pose to the robot's pose estimator.
   */
  public void estimateRobotPose() {
    setPipeline(Pipeline.localizeRobot);
    PoseEstimate measurement = getPoseEstimate();
    if (measurement.tagCount >= 1) {
      SmartDashboard.putNumber("Limelight/" + name + "/Distance from Odometry", getOdometryDifference(measurement.pose));
      
      double xyStds = calculateXYStdDevs2(measurement);
      SmartDashboard.putNumber("Limelight/" + name + "/XY Std Devs", xyStds);

      RobotContainer.drive.drive.addVisionMeasurement(
        // ignore rotation
        new Pose2d(measurement.pose.getTranslation(), RobotContainer.drive.drive.getOdometryHeading()),
        measurement.timestampSeconds,
        VecBuilder.fill(xyStds, xyStds, 1)
      );
    }
  }

  /**
   * Find the number of targets without parsing the JSON the Limelight puts on NT.
   * @param printTime Prints the elapsed time if true.
   * @return The amount of targets detected.
   */
  public int getNumTargetsFast(boolean printTime) {
    String jsonDump = LimelightHelpers.getJSONDump(name);
    double start = Timer.getFPGATimestamp();
    Pattern pattern = Pattern.compile("\"fID\":\\d+");
    Matcher matcher = pattern.matcher(jsonDump);

    int count = 0;
    while (matcher.find()) {
      count++;
    }

    if (printTime) {
      System.out.println("Limelight " + name + " NumTargets DT: " + (Timer.getFPGATimestamp() - start));
    }
    return count;
  }

  /**
   * Get the current pose estimate from LimelightHelpers.
   * <p>Also logs some data to SmartDashboard.
   * @return Current pose estimate.
   */
  public PoseEstimate getPoseEstimate() {
    PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", measurement.tagCount >= 1);
    RobotContainer.drive.drive.field.getObject(name + " Estimated Pose").setPose(measurement.pose);
    return measurement;
  }

  /**
   * Check if the provided pose estimate is valid and within the field.
   * @return If the provided pose estimate is valid.
   */
  public boolean isPoseValid() {
    // TODO: still sketchy, as pose can change between calls to getPoseEstimate()
    var measurement = getPoseEstimate();
    return poseWithinField(measurement.pose) && measurement.tagCount >= 1;
  }

  /**
   * Reset the robot pose to the detected pose if it is valid.
   */
  public void resetRobotPose() {
    setPipeline(Pipeline.localizeRobot);
    PoseEstimate measurement = getPoseEstimate();
    if (measurement.tagCount >= 1) {
      RobotContainer.drive.drive.resetOdometry(measurement.pose);
    }
  }

  /**
   * Set this limelight's pipeline through NetworkTables.
   * @param pipeline The pipeline to set.
   */
  public void setPipeline(Pipeline pipeline) {
    NetworkTableInstance.getDefault().getTable(name).getEntry("pipeline").setNumber(pipeline.id);
  }

  // --- Command Factories ---

  /**
   * Get a command that adds a pose estimate to the robot's pose estimator repeatedly until it
   * ends. Runs when disabled.
   * @return A command that adds pose estimates from the Limelight until interrupted.
   */
  public Command getEstimateRobotPose() {
    return new RunCommand(this::estimateRobotPose, this)
      .ignoringDisable(true);
  }

  /**
   * Get a command that resets the robot pose to the detected pose (if valid) and then ends. Runs
   * when disabled.
   * @return A command that resets the robot pose once and then ends.
   */
  public Command getResetRobotPose() {
    return new InstantCommand(this::resetRobotPose, this)
      .ignoringDisable(true);
  }

  // --- Static Functions ---

  /**
   * Calculate the distance between the current vision and odometry poses.
   * @param pose Estimated Vision Pose
   * @return Linear distance between vision and odometry poses.
   */
  public static double getOdometryDifference(Pose2d pose) {
    var odometryDifference = RobotContainer.drive.getPoseEfficiently().minus(pose);
    return Math.hypot(odometryDifference.getX(), odometryDifference.getY());
  }

  /**
   * Check whether pose is within game field.
   * @param pose Pose to check.
   * @return Whether pose is within game field.
   */
  public static boolean poseWithinField(Pose2d pose) {
    return (pose.getX() >= 0 && pose.getX() <= AprilTag.fieldLayout.getFieldLength())
      && (pose.getY() >= 0 && pose.getY() <= AprilTag.fieldLayout.getFieldWidth());
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
