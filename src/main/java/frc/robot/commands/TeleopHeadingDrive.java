// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.Xbox;

/**
 * Drive the robot using translational velocity from the driver controller and
 * match the robot angle to the rotation joystick X axis.
 * <p>Left bumper slows robot down, right bumper speeds it up.
 * Precision/Boost amount can be adjusted through NetworkTables.
 */
public class TeleopHeadingDrive extends Command {
  private final boolean isFieldRelative;

  /** Creates a new TeleopHeadingDrive. */
  public TeleopHeadingDrive(boolean isFieldRelative) {
    this.isFieldRelative = isFieldRelative;
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = RobotContainer.drive.drive.swerveController.getTargetSpeeds(
      // controller is +ve backwards, field coordinates are +ve forward
      MathUtil.applyDeadband(-RobotContainer.driverController.getLeftY(), Xbox.joystickDeadband),
      // controller is +ve right, field coordinates are +ve left
      MathUtil.applyDeadband(-RobotContainer.driverController.getLeftX(), Xbox.joystickDeadband),
      // controller is +ve right (CW+), YAGSL expects CCW+ (+ve left)
      MathUtil.applyDeadband(-RobotContainer.driverController.getRightX() * Math.PI, Xbox.joystickDeadband), // -PI to PI radians
      RobotContainer.drive.drive.getYaw().getRadians(),
      RobotContainer.drive.getTeleopMaxLinearSpeed()
    );
    
    if (isFieldRelative) {
      RobotContainer.drive.drive.driveFieldOriented(speeds);
    } else {
      RobotContainer.drive.drive.drive(speeds);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
