// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTag.ID;
import frc.robot.subsystems.Limelight;

/**
 * Calculate shooter angle using the robot's distance from the speaker.
 * Assumes odometry is accurate and projectile flies in a straight line.
 */
public class AutoSetAngle extends Command {
  // Units in meters, positive = up or forward
  public static final double speakerHeight = 2.18;
  public static final double heightOffset = 0.08;
  public static final double noteHeight = 0.19; // distance from ground to note at shooter pivot
  public static final double shooterDistanceFromCenter = -0.02;
  public static final double targetHeight = speakerHeight + heightOffset - noteHeight;

  /** Creates a new AutoSetAngle. */
  public AutoSetAngle() {
    addRequirements(RobotContainer.shooterAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Pose2d robotPose = RobotContainer.drive.drive.getPose();
    final ID speakerID = RobotContainer.isRedAlliance("AutoSetAngle") ? ID.kRedSpeakerCenter : ID.kBlueSpeakerCenter;
    final double distanceToSpeaker = robotPose.getTranslation().getDistance(Limelight.getTargetPose2d(speakerID).getTranslation());
    final double pivotDistanceToSpeaker = distanceToSpeaker + shooterDistanceFromCenter;
    final Rotation2d angle = Rotation2d.fromRadians(Math.atan(targetHeight / pivotDistanceToSpeaker));

    SmartDashboard.putNumber("AutoSetAngle/Distance to Shooter Pivot (m)", pivotDistanceToSpeaker);
    SmartDashboard.putNumber("AutoSetAngle/Calculated Angle (deg)", angle.getDegrees());

    RobotContainer.shooterAngle.setAngle(angle);
  }
}
