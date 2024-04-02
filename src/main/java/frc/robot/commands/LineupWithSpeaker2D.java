// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTag;
import frc.robot.Constants.AprilTag.ID;
import frc.robot.utils.LimeLightHelpers;

/**
 * Align robot heading to AprilTag according to automatically-detected alliance colour.
 * Works standalone or alongside a command controlling robot translation (or PathPlanner).
 */
public class LineupWithSpeaker2D extends Command {
  private Timer odometryValidTimer = new Timer();
  private boolean aligningToTag = false;
  // exists since 'isScheduled()' doesn't work when command is scheduled as part of a group
  private boolean isRunning = false;

  /**
   * Creates a new LineupWithSpeaker2D.
   * This class creates a command that turns the robot towards the speaker AprilTag using 2D tracking
   */
  public LineupWithSpeaker2D() {
    // does not require drive since this is intended to be used in addition to a command controlling translation
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    odometryValidTimer.restart();
    isRunning = true;
    // Switch to 2D pipeline
    // Note: the tag filter should be set to IDs 4 and 7
    LimeLightHelpers.setPipelineIndex("limelight-shooter", 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d heading;
    if(LimeLightHelpers.getTV("limelight-shooter")){
      aligningToTag = true;
      heading = RobotContainer.drive.drive.getOdometryHeading().plus(Rotation2d.fromDegrees(-LimeLightHelpers.getTX("limelight-shooter")));
      odometryValidTimer.restart();
    }else if(!odometryValidTimer.hasElapsed(0.5)){
      // if we lose heading, wait for a bit before reverting to gyro
      aligningToTag = false;
      // stay at current heading and wait for the limelight to give an angle again
      heading = RobotContainer.drive.getPoseEfficiently().getRotation();
    }else{
      // limelight doesn't see tag -> turn robot towards the wall
      heading = new Rotation2d(0);
      aligningToTag = false;
    }
    RobotContainer.drive.overrideHeading(heading);
    SmartDashboard.putNumber("AprilTagTrack/Target Heading (Deg)", heading.getDegrees());
    SmartDashboard.putBoolean("AprilTagTrack/Aligning to Odometry", aligningToTag);
    SmartDashboard.putBoolean("AprilTagTrack/At Target", atTarget());

  }

  @Override
  public void end(boolean interrupted) {
    odometryValidTimer.stop();
    isRunning = false;
  }

  public boolean atTarget() {
    return isRunning && aligningToTag && RobotContainer.drive.drive.swerveController.thetaController.atSetpoint();
  }

}
