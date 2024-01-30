// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.Xbox;

/**
 * Drive the robot using translational velocity from the driver controller and
 * match the robot angle to the rotation joystick X axis.
 * <p>Left bumper slows robot down, right bumper speeds it up.
 * Precision/Boost amount can be adjusted through NetworkTables.
 */
public class VisionTrackDrive extends Command {
  private final double verticalAngle = 38.0; // limelight angle down
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry txEntry = limelightTable.getEntry("tx");
  private final NetworkTableEntry tyEntry = limelightTable.getEntry("ty");
  /** Creates a new TeleopHeadingDrive. */
  public VisionTrackDrive() {
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tx = txEntry.getDouble(-2609);
    double ty = tyEntry.getDouble(-2609); // range [-15,20]
    double forward = (ty+18)/38; // normalize to [0,1]
    double xSpeed = MathUtil.applyDeadband(-RobotContainer.driverController.getRightTriggerAxis(), Xbox.joystickDeadband)*forward; // "Deadman's switch"
    if(tx == -2609){
      DriverStation.reportError("Limelight not connected", null);
      return;
    }
    if(ty == -2609){
      DriverStation.reportError("Limelight not connected", null);
      xSpeed = 0;
    }

    
    ChassisSpeeds speeds = RobotContainer.drive.drive.swerveController.getTargetSpeeds(
      // controller is +ve backwards, field coordinates are +ve forward
      xSpeed,
      // controller is +ve right, field coordinates are +ve left
      MathUtil.applyDeadband(-RobotContainer.driverController.getLeftX(), Xbox.joystickDeadband),
      // controller is +ve right (CW+), YAGSL expects CCW+ (+ve left)
      RobotContainer.drive.drive.getYaw().getRadians()+Math.toRadians(tx), // -PI to PI radians
      RobotContainer.drive.drive.getYaw().getRadians(),
      RobotContainer.drive.getTeleopMaxLinearSpeed()
    );
    RobotContainer.drive.drive.drive(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Vision track done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
