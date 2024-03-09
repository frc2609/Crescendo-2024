// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTag.ID;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterFlywheel.SpinType;

/**
 * Calculate shooter angle and RPM using the robot's distance from the speaker.
 * Assumes odometry is accurate and projectile flies in a straight line at calculated RPM.
 * Works the best within alliance area.
 */
public class AutoSetShooter extends Command {
  // Units in meters, positive = up or forward
  public static final double speakerHeight = 1.98;
  public static final double heightOffset = 0.125;
  public static final double noteHeight = 0.19; // distance from ground to note at shooter pivot
  public static final double shooterDistanceFromCenter = -0.02;
  public static final double targetHeight = speakerHeight + heightOffset - noteHeight;

  // generate a linear equation that passes through these two points
  // distance for RPM is measured from the center of the robot
  public static final double closeDistance = 1.4;
  public static final double closeRPM = 3000;
  public static final double farDistance = 5.0;
  public static final double farRPM = 5800;
  public static final double rpmEquationSlope = (farRPM - closeRPM) / (farDistance - closeDistance);

  private final SpinType spinType;
  private Translation2d speakerTranslation;

  /** Creates a new AutoSetShooter. */
  public AutoSetShooter(SpinType spinType) {
    this.spinType = spinType;
    addRequirements(RobotContainer.shooterAngle, RobotContainer.shooterFlywheel);
  }

  @Override
  public void initialize() {
    final ID speakerID = RobotContainer.isRedAlliance("AutoSetAngle") ? ID.kRedSpeakerCenter : ID.kBlueSpeakerCenter;
    speakerTranslation = Limelight.getTargetPose2d(speakerID).getTranslation();
  }

  @Override
  public void execute() {
    final Pose2d robotPose = RobotContainer.drive.drive.getPose();
    final double distanceToSpeaker = robotPose.getTranslation().getDistance(speakerTranslation);
    final double pivotDistanceToSpeaker = distanceToSpeaker + shooterDistanceFromCenter;
    
    final Rotation2d angle = Rotation2d.fromRadians(Math.atan(targetHeight / pivotDistanceToSpeaker));
    final double rpm = rpmEquationSlope * (distanceToSpeaker - closeDistance) + closeRPM;

    SmartDashboard.putNumber("AutoSetShooter/Distance to Shooter Pivot (m)", pivotDistanceToSpeaker);
    SmartDashboard.putNumber("AutoSetShooter/Calculated Angle (deg)", angle.getDegrees());
    SmartDashboard.putNumber("AutoSetShooter/Calculated RPM", rpm);

    RobotContainer.shooterAngle.setAngle(angle);
    RobotContainer.shooterFlywheel.setSpeed(rpm, spinType);
  }
}