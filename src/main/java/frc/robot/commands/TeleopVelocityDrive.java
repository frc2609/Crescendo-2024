// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.Xbox;

/**
 * Drive the robot using translational and rotational velocity from the driver
 * controller.
 * <p>Left bumper slows robot down, right bumper speeds it up.
 * Precision/Boost amount can be adjusted through NetworkTables.
 */
public class TeleopVelocityDrive extends Command {
  private final boolean isFieldRelative;

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
    // controller is +ve backwards, field coordinates are +ve forward
    final double desiredXTranslation = MathUtil.applyDeadband(-RobotContainer.driverController.getLeftY(), Xbox.joystickDeadband);
    // controller is +ve right, field coordinates are +ve left
    final double desiredYTranslation = MathUtil.applyDeadband(-RobotContainer.driverController.getLeftX(), Xbox.joystickDeadband);
    // controller is +ve right (CW+), YAGSL expects CCW+ (+ve left)
    final double desiredAngularVelocity = MathUtil.applyDeadband(-RobotContainer.driverController.getRightX(), Xbox.joystickDeadband);

    RobotContainer.drive.drive.drive(
      new Translation2d(desiredXTranslation, desiredYTranslation).times(RobotContainer.drive.getTeleopMaxLinearSpeed()),
      desiredAngularVelocity * RobotContainer.drive.getMaxAngularSpeed(),
      isFieldRelative,
      false
    );
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
