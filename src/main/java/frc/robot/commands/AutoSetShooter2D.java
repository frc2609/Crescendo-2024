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
import frc.robot.Constants.AprilTag;
import frc.robot.Constants.AprilTag.ID;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LED.BlinkMode;
import frc.robot.subsystems.LED.Pattern;
import frc.robot.subsystems.ShooterFlywheel.SpinType;
import frc.robot.utils.LimeLightHelpers;

/**
 * Calculate shooter angle and RPM using the robot's distance from the speaker.
 * Assumes odometry is accurate and projectile flies in a straight line at calculated RPM.
 * Works the best within alliance area.
 * Does not stop shooter when complete (call 'IdleShooter' if desired).
 */
public class AutoSetShooter2D extends Command {
  // Units in meters, positive = up or forward
  public static final double speakerHeight = 1.98;
  public static double heightOffset = 0; // 0.7
  public static final double noteHeight = 0.19; // distance from ground to note at shooter pivot
  public static final double shooterDistanceFromCenter = -0.02;
  public static double targetHeight = speakerHeight + heightOffset - noteHeight;

  // generate a linear equation that passes through these two points
  // distance for RPM is measured from the center of the robot
  public static final double closeDistance = 1.4;
  public static final double closeRPM = 3500;
  public static final double farDistance = 5.0;
  public static final double farRPM = 5800;
  public static final double rpmEquationSlope = (farRPM - closeRPM) / (farDistance - closeDistance);
  public static final double limelightHeight = 0.64;
  public static final double heightToTag = 1.343 - limelightHeight; // speaker tag height - limelight height (both from floor)

  private final SpinType spinType;
  // exists since 'isScheduled()' doesn't work when command is scheduled as part of a group
  private boolean isRunning = false;

  /** Creates a new AutoSetShooter. */
  public AutoSetShooter2D(SpinType spinType) {
    this.spinType = spinType;
    addRequirements(RobotContainer.shooterAngle, RobotContainer.shooterFlywheel);
    SmartDashboard.putNumber("Height Offset", heightOffset);
  }

  @Override
  public void initialize() {
    isRunning = true;
    LimeLightHelpers.setPipelineIndex("limelight-shooter", 1);
  }

  @Override
  public void execute() {
    double distanceToSpeaker;
    if(LimeLightHelpers.getTV("limelight-shooter")){
      isRunning = true;
      distanceToSpeaker = heightToTag/Math.tan(Math.toRadians(LimeLightHelpers.getTY("limelight-shooter")+15)); // 15 deg = limelight tilt angle from horizon
      
      final double pivotDistanceToSpeaker = distanceToSpeaker + shooterDistanceFromCenter;
      heightOffset = SmartDashboard.getNumber("Height Offset", heightOffset);
      targetHeight = speakerHeight + heightOffset - noteHeight;
    
      final Rotation2d angle = Rotation2d.fromRadians(Math.atan(targetHeight / pivotDistanceToSpeaker));
      final double rpm = rpmEquationSlope * (distanceToSpeaker - closeDistance) + closeRPM;

      
      RobotContainer.shooterAngle.setAngle(angle);
      RobotContainer.shooterFlywheel.setSpeed(rpm, spinType);
      
      SmartDashboard.putNumber("AutoSetShooter/Distance to Shooter Pivot (m)", pivotDistanceToSpeaker);
      SmartDashboard.putNumber("AutoSetShooter/Calculated Angle (deg)", angle.getDegrees());
      SmartDashboard.putNumber("AutoSetShooter/Calculated RPM", rpm);
    }else{
      // No target found
      isRunning = false;
    }

    SmartDashboard.putBoolean("AutoSetShooter/At Target", atTarget());

  }

  @Override
  public void end(boolean interrupted) {
    isRunning = false;
  }

  public boolean atTarget() {
    return isRunning && RobotContainer.shooterAngle.atTarget() && RobotContainer.shooterFlywheel.atSetSpeed();
  }
}