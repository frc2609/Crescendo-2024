// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drive extends SubsystemBase {
  public final SwerveDrive drive;
  private final double originalMaxAngularSpeed;

  /** Creates a new Drive. */
  public Drive(boolean verboseTelemetry) {
    if (verboseTelemetry) {
      SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    }

    try {
      drive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
        .createSwerveDrive(
          Swerve.maxModuleSpeed,
          Swerve.angleConversionFactor,
          Swerve.driveConversionFactor
        );
      originalMaxAngularSpeed = drive.getMaximumAngularVelocity();
    } catch (IOException e) {
      DataLogManager.log("Swerve Drive File Error: " + e.getMessage());
      throw new RuntimeException("Swerve Drive failed to initialize.");
    }

    // setup PathPlanner
    AutoBuilder.configureHolonomic(
      drive::getPose,
      drive::resetOdometry,
      drive::getRobotVelocity,
      drive::drive,
      new HolonomicPathFollowerConfig(
          new PIDConstants(5.0, 0.0, 0.0),
          new PIDConstants(5.0, 0.0, 0.0),
          // limit speeds in the paths, NOT HERE.
          drive.getMaximumVelocity(),
          // Calculate drivetrain radius
          Math.sqrt(Math.pow(drive.swerveDriveConfiguration.moduleLocationsMeters[0].getY(), 2) + Math.pow(drive.swerveDriveConfiguration.moduleLocationsMeters[0].getX(), 2)),
          new ReplanningConfig() // customize this as desired
      ),
      () -> {
        // mirror the path (drawn on blue side) if we are on the red alliance
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );

    SmartDashboard.putNumber("swerve/teleop/linearSpeedMultiplier", 1.0);
    SmartDashboard.putNumber("swerve/teleop/angularSpeedMultiplier", 1.0);
  }

  /**
   * Use this to scale linear speed when controlling the robot in teleop.
   * @return The maximum speed the robot can go multiplied by teleop speed multiplier.
   */
  public double getLimitedTeleopLinearSpeed() {
    return drive.getMaximumVelocity() * SmartDashboard.getNumber("swerve/teleop/linearSpeedMultiplier", 1.0);
  }

  /**
   * Use this to scale angular speed when controlling the heading by angular speed.
   * @return The maximum speed the robot can spin multiplied by teleop speed multiplier.
   */
  public double getLimitedTeleopAngularSpeed() {
    return originalMaxAngularSpeed * SmartDashboard.getNumber("swerve/teleop/angularSpeedMultiplier", 1.0);
  }

  @Override
  public void periodic() {
    // limits the angular speed when the robot is controlled through a heading
    drive.swerveController.setMaximumAngularVelocity(getLimitedTeleopAngularSpeed());
  }
}
