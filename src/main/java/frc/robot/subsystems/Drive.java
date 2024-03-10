// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Swerve;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drive extends SubsystemBase {
  private final Rotation2d headingTolerance = Rotation2d.fromDegrees(4);
  private final double originalMaxAngularSpeed;

  /** All YAGSL functionality is implemented in this instance of SwerveDrive. */
  public final SwerveDrive drive;
  private Optional<Rotation2d> headingOverride = Optional.empty();

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

    drive.swerveController.thetaController.setTolerance(headingTolerance.getRadians());

    // setup PathPlanner
    AutoBuilder.configureHolonomic(
      drive::getPose,
      drive::resetOdometry,
      drive::getRobotVelocity,
      // PathPlanner supplies *robot-relative* chassisSpeeds
      (ChassisSpeeds speeds) -> setChassisSpeeds(speeds, false),
      new HolonomicPathFollowerConfig(
          new PIDConstants(5.0, 0.0, 0.0),
          // 'i' is highly not recommended since the heading can be overidden causing 'i' to build up
          new PIDConstants(5.0, 0.0, 0.0),
          // limit speeds in the paths, NOT HERE.
          drive.getMaximumVelocity(),
          // Calculate drivetrain radius
          Math.sqrt(Math.pow(drive.swerveDriveConfiguration.moduleLocationsMeters[0].getY(), 2) + Math.pow(drive.swerveDriveConfiguration.moduleLocationsMeters[0].getX(), 2)),
          new ReplanningConfig() // customize this as desired
      ),
      () -> {
        return RobotContainer.isRedAlliance("Drive (PathPlanner)");
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

  /**
   * Override PathPlanner's heading while leaving translation alone.
   * Must be called each loop cycle to function.
   * Once it stops being called, automatically returns heading control to PathPlanner.
   * @param headingOverride The Rotation2d to set the robot's heading to.
   */
  public void overrideHeading(Rotation2d headingOverride) {
    this.headingOverride = Optional.of(headingOverride);
  }

  /**
   * Wrapper for SwerveDrive::drive that allows the rotational velocity to be overridden.
   * This is the only drive function you should call.
   * @param chassisSpeeds Desired ChassisSpeeds.
   * @param isFieldRelative Whether to use 'chassisSpeeds' as field relative speeds or robot relative speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isFieldRelative) {
    if (headingOverride.isPresent()) {
      SmartDashboard.putNumber("swerve/headingOverride (deg)", headingOverride.get().getDegrees());
      chassisSpeeds.omegaRadiansPerSecond = drive.swerveController.headingCalculate(
        drive.getYaw().getRadians(),
        headingOverride.get().getRadians()
      );
      // empty the optional so heading control is returned unless the heading is overridden in the next loop cycle
      this.headingOverride = Optional.empty();
    }

    if (isFieldRelative) {
      drive.driveFieldOriented(chassisSpeeds);
    } else {
      drive.drive(chassisSpeeds);
    }
  }
}
