// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetAngleToDistance extends Command {
  /** Creates a new SetAngleToDistance. */
  public SetAngleToDistance() {
    addRequirements(RobotContainer.shooterAngle);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooterAngle.setAngle(getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public Rotation2d getAngle() {
    double bumper = 3;
    double base = 36.5;
    // double shooterToEdge = 13.5;
    double startingPoseOffset = bumper + base + 14.25; // half frame dist
    // 14.25 = half frame distance, 13.5 = distance from frame to center
    double shooterToCenter = -(14.25 - 13.5); // shooter is 1.25 inches behind robot center (where pose2d starts)
    // 13.5 = pivot distance from robot frame lol
    // with actual code, should be distance from sensor (want to find shooter pivot distance from speaker)
    // double distance = Units.inchesToMeters(SmartDashboard.getNumber("distance from base (inches)", 0) + 36.5 + 13.5);
    double distance = RobotContainer.drive.drive.getPose().getX() + Units.inchesToMeters(startingPoseOffset - shooterToCenter);
    SmartDashboard.putNumber("odometry distance (inches)", Units.metersToInches(distance));
    Rotation2d calculatedAngle = Rotation2d.fromRadians(Math.atan(2.18 / distance)); // 2.18 = speaker target
    // System.out.println("hi there I am running");
    System.out.println(distance);
    SmartDashboard.putNumber("calc'd angle degrees", calculatedAngle.getDegrees());
    return calculatedAngle;
  }
}
