// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.DriveUtil;
import frc.robot.utils.LimeLightHelpers;
import frc.robot.utils.TunableNumber;

/**
 * TODO: Actual docs
 */
public class VisionTrackDrive extends Command {
  private final boolean isFieldRelative;
  private double tx;

  TunableNumber kP = new TunableNumber("kP", 0.05);
  TunableNumber kI = new TunableNumber("kI", 0.0);
  TunableNumber kD = new TunableNumber("kD", 0.0);

  PIDController headingPID = new PIDController(kP.get(), kI.get(), kD.get());

  /** Creates a new VisionTrackDrive. */
  public VisionTrackDrive(boolean isFieldRelative) {
    addRequirements(RobotContainer.drive);
    this.isFieldRelative = isFieldRelative;
    // SmartDashboard.putData("Heading PID", headingPID); // TODO: look into how to do this

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tx = LimeLightHelpers.getTX("limelight");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = LimeLightHelpers.getTX("limelight");

    if (kP.hasChanged(hashCode())) {
      headingPID.setP(kP.get());
    }

    if (kI.hasChanged(hashCode())) {
      headingPID.setI(kI.get());
    }
    
    if (kD.hasChanged(hashCode())) {
      headingPID.setD(kD.get());
    }

    double calculatedAngularVelocity = headingPID.calculate(tx);

    // TODO: add this to a table (e.g. vision/)
    SmartDashboard.putNumber("calculatedAngularVelocity", calculatedAngularVelocity);

    double[] driverInputs = DriveUtil.getDriverInputs(
      RobotContainer.driverController,
      true,
      false,
      false,
      true,
      DriveUtil.getSensitivity(RobotContainer.driverController)
    );

    RobotContainer.drive.drive.drive(
      new Translation2d(driverInputs[0], driverInputs[1])
        .times(RobotContainer.drive.getLimitedTeleopLinearSpeed()),
      calculatedAngularVelocity,
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
