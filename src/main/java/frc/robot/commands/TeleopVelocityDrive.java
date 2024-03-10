// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.DriveUtil;

/**
 * Drive the robot using translational and angular velocity from the driver controller.
 */
public class TeleopVelocityDrive extends Command {
  public final boolean isFieldRelative;

  /** Creates a new TeleopVelocityDrive. */
  public TeleopVelocityDrive(boolean isFieldRelative) {
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
      false,
      false,
      true,
      DriveUtil.getSensitivity(RobotContainer.driverController),
      isFieldRelative
    );

    ChassisSpeeds speeds = new ChassisSpeeds(
      driverInputs[0] * RobotContainer.drive.getLimitedTeleopLinearSpeed(),
      driverInputs[1] * RobotContainer.drive.getLimitedTeleopLinearSpeed(),
      driverInputs[2] * RobotContainer.drive.getLimitedTeleopAngularSpeed()
    );

    RobotContainer.drive.setChassisSpeeds(speeds, isFieldRelative);
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
