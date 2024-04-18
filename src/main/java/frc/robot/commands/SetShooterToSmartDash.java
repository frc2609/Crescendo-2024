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
import frc.robot.subsystems.ShooterFlywheel.SpinType;

/**
 * Set the shooter to a desired preset angle, RPM, and spin.
 */
public class SetShooterToSmartDash extends Command {
  

  /** Creates a new SetShooterToPreset. */
  public SetShooterToSmartDash() {
    addRequirements(RobotContainer.shooterAngle, RobotContainer.shooterFlywheel);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    
    RobotContainer.shooterAngle.setAngle(new Rotation2d(Math.toRadians(SmartDashboard.getNumber("Angle setp", 20.0))));
    if(SmartDashboard.getBoolean("spin enabled", true)){
      RobotContainer.shooterFlywheel.setSpeed(SmartDashboard.getNumber("RPM setp", 3500.0), SpinType.slowRightMotor);
    }else{
      RobotContainer.shooterFlywheel.setSpeed(SmartDashboard.getNumber("RPM setp", 3500.0), SpinType.disable);
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterAngle.stop();
    RobotContainer.shooterFlywheel.coast();
  }
}
