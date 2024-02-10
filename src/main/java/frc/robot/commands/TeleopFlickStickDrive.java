// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.DriveUtil;

/**
 * Drive the robot using translational velocity from the driver controller and match the robot
 * angle to the angle of the rotation joystick.
 */
public class TeleopFlickStickDrive extends Command {
  public final boolean isFieldRelative;

  /** Creates a new TeleopFlickStickDrive. */
  public TeleopFlickStickDrive(boolean isFieldRelative) {
    this.isFieldRelative = isFieldRelative;
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] driverInputs = DriveUtil.getDriverInputs(
      RobotContainer.driverController,
      true,
      true,
      // if not field relative, cube both inputs
      // if field relative, don't cube y so manuvering left/right is more responsive at low speeds
      !isFieldRelative,
      false,
      DriveUtil.getSensitivity(RobotContainer.driverController)
    );

    ChassisSpeeds speeds = RobotContainer.drive.drive.swerveController.getTargetSpeeds(
      driverInputs[0],
      driverInputs[1],
      driverInputs[3],
      driverInputs[4],
      RobotContainer.drive.drive.getYaw().getRadians(),
      RobotContainer.drive.getLimitedTeleopLinearSpeed()
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
