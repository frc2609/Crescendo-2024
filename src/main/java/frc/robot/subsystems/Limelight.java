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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTag;
import frc.robot.utils.LimeLightHelpers;

public class Limelight extends SubsystemBase {
  public static enum Pipeline {
    // front limelight
    noteDetection(0),
    // other limelights
    localizeRobot(0),
    track2d(1);

    public int id;

    private Pipeline(int id) {
      this.id = id;
    }
  }
  private final String name;

  /** Creates a new Limelight. */
  public Limelight(String name) {
    this.name = name;
    SmartDashboard.putNumber("Limelight/" + name + "/Distance Std Devs", 0.1);
    SmartDashboard.putNumber("Limelight/" + name + "/Velocity Std Devs", 0.5);
    // create the limelight pose object immediately instead of the first time it is used
    RobotContainer.drive.drive.field.getObject(name + " Estimated Pose");
  }

  @Override
  public void periodic() {}

  /**
   * Calculate standard deviations for the robot's pose estimator based on the accuracy of the
   * current detected pose.
   * @return [x/y std devs, rotation std devs]
   */
  public double[] calculateStdDevs() {
    double distanceStdDevMultiplier = SmartDashboard.getNumber("Limelight/" + name + "/Distance Std Devs", 0.1);
    double velocityStdDevMultiplier = SmartDashboard.getNumber("Limelight/" + name + "/Velocity Std Devs", 0.5);
    double tagArea = LimeLightHelpers.getTA(name); // SKETCHY
    double xyStds = distanceStdDevMultiplier * (1 / tagArea) + velocityStdDevMultiplier * RobotContainer.drive.getVelocity();
    double rotStds = 20;
    SmartDashboard.putNumber("Limelight/" + name + "/TA", tagArea);
    SmartDashboard.putNumber("Limelight/" + name + "/XY STDS", xyStds);
    SmartDashboard.putNumber("Limelight/" + name + "/ROT STDS", rotStds);
    return new double[] { xyStds, rotStds };
  }

  /**
   * Add the current detected pose to the robot's pose estimator.
   */
  public void estimateRobotPose() {
    setPipeline(Pipeline.localizeRobot);
    var detectedPose = getPose();
    if (isPoseValid(detectedPose)) {
      double latencyMS = LimeLightHelpers.getLatency_Capture(name) / 1000.0;
      double[] stdDevs = calculateStdDevs();

      RobotContainer.drive.drive.addVisionMeasurement(new Pose2d(detectedPose.get().getTranslation(), RobotContainer.drive.drive.getOdometryHeading()), Timer.getFPGATimestamp() - latencyMS, VecBuilder.fill(stdDevs[0], stdDevs[0], Units.degreesToRadians(stdDevs[1])));
      
      SmartDashboard.putNumber("Limelight/" + name + "/Latency (MS)", latencyMS);
      SmartDashboard.putNumber("Limelight/" + name + "/Odometry Error", getOdometryDifference(detectedPose.get()));
      SmartDashboard.putNumber("Limelight/" + name + "/Total Target Area", LimeLightHelpers.getTA(name));
    }
  }

  /**
   * Find the number of targets without parsing the JSON the Limelight puts on NT.
   * @param printTime Prints the elapsed time if true.
   * @return The amount of targets detected.
   */
  public int getNumTargetsFast(boolean printTime) {
    String jsonDump = LimeLightHelpers.getJSONDump(name);
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
   * Get the current detected pose. May or may not be valid.
   * @return The current detected robot pose if valid or Optional.empty() if not.
   */
  public Optional<Pose2d> getPose() {
    // TODO: check in periodic and set to member variable (instead of potentially calling multiple times per loop)
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

  /**
   * Check if the current detected pose is valid.
   * @return If the current detected pose is valid.
   */
  public boolean isPoseValid() {
    // sketchy, could be valid when called but invalid when actually detecting
    // TODO: record pose to member then check that for validity
    return isPoseValid(getPose());
  }

  /**
   * Reset the robot pose to the detected pose if it is valid.
   */
  public void resetRobotPose() {
    setPipeline(Pipeline.localizeRobot);
    if (!LimeLightHelpers.getTV(name)) { 
      SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", false);
      return;
    }
    SmartDashboard.putBoolean("Limelight/" + name + "/Pose Valid", true);
    var pose = LimeLightHelpers.getBotPose2d_wpiBlue(name);

    RobotContainer.drive.drive.field.getObject(name + " Estimated Pose").setPose(pose);
    RobotContainer.drive.drive.resetOdometry(pose);
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
   * 
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
