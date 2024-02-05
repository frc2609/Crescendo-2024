// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public void initialize() {
    RobotContainer.drive.drive.setHeadingCorrection(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // controller is +ve backwards, field coordinates are +ve forward
    double desiredXTranslation = MathUtil.applyDeadband(-RobotContainer.driverController.getLeftY(), Xbox.joystickDeadband);
    // controller is +ve right, field coordinates are +ve left
    double desiredYTranslation = MathUtil.applyDeadband(-RobotContainer.driverController.getLeftX(), Xbox.joystickDeadband);
    // controller is +ve right (CW+), YAGSL expects CCW+ (+ve left)
    double desiredAngularVelocity = MathUtil.applyDeadband(-RobotContainer.driverController.getRightX(), Xbox.joystickDeadband);

    double actualX = desiredXTranslation;
    double actualY = desiredYTranslation;

    // change these booleans to cube/correct in different orders if you want to test the effects
    // described in Swerve Drive Control.md
    // (this will be removed next commit)
    if (false) {
      actualX = Math.pow(desiredXTranslation, 3);
      actualY = Math.pow(desiredYTranslation, 3);
    }

    double[] correctedTranslation;
    if (true) {
      SmartDashboard.putNumber("uncorrected x", desiredXTranslation);
      SmartDashboard.putNumber("uncorrected y", desiredYTranslation);
      SmartDashboard.putNumber("uncorrected mag", Math.sqrt(desiredXTranslation*desiredXTranslation + desiredYTranslation*desiredYTranslation));
      correctedTranslation = RobotContainer.drive.correctForSquareJoystickMapping(desiredXTranslation, desiredYTranslation);
      SmartDashboard.putNumber("corrected x", correctedTranslation[0]);
      SmartDashboard.putNumber("corrected y", correctedTranslation[1]);
      SmartDashboard.putNumber("corrected mag", Math.sqrt(correctedTranslation[0]*correctedTranslation[0] + correctedTranslation[1]*correctedTranslation[1]));
      actualX = correctedTranslation[0];
      actualY = correctedTranslation[1];
    }

    if (true) {
      actualX = Math.pow(correctedTranslation[0], 3);
      actualY = Math.pow(correctedTranslation[1], 3);
    }
    
    RobotContainer.drive.drive.drive(
      new Translation2d(actualX, actualY).times(RobotContainer.drive.getTeleopMaxLinearSpeed()),
      desiredAngularVelocity * RobotContainer.drive.getMaxAngularSpeed(),
      isFieldRelative,
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drive.drive.setHeadingCorrection(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
