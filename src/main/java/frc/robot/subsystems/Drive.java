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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.RobotContainer;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.TunableNumber;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drive extends SubsystemBase {
  /** Multiplier to apply to overall max speed. */
  public final TunableNumber linearSpeedMultiplier = new TunableNumber("swerve/teleop/Linear Speed Multiplier", 1.0);
  /** Multiplier to apply to max angular speed. */
  public final TunableNumber angularSpeedMultiplier = new TunableNumber("swerve/teleop/Angular Speed Multiplier", 1.0);
  /** Affects teleop max speed when not in boost or precision mode. */
  private final TunableNumber normalMultiplier = new TunableNumber("swerve/teleop/Normal Speed Multiplier", 0.9);
  /** How much to increase the speed multiplier in getTeleopMaxLinearSpeed() when in boost mode. */
  private final TunableNumber boostIncrease = new TunableNumber("swerve/teleop/Boost Speed Multiplier Increase", 0.1);
  /** How much to decrease the speed multiplier in getTeleopMaxLinearSpeed() when in precision mode. */
  private final TunableNumber precisionReduction = new TunableNumber("swerve/teleop/Precision Speed Multiplier Reduction", 0.6);
  public final double originalMaxAngularVelocity;
  public final SwerveDrive drive;
  private final BeaverLogger logger = new BeaverLogger();

  /** Creates a new Drive. */
  public Drive() {
    // uncomment to add human-readable data to NetworkTables
    // SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    try {
      drive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(getMaxLinearSpeed(), Swerve.angleConversionFactor, Swerve.driveConversionFactor);
      // drive.setMaximumSpeeds(getMaxLinearSpeed(), getMaxLinearSpeed(), getMaxAngularSpeed());
      originalMaxAngularVelocity = drive.getSwerveController().config.maxAngularVelocity;
    } catch (IOException e) {
      DataLogManager.log("Swerve Drive File Error: " + e.getMessage());
      throw new RuntimeException("Swerve Drive failed to initialize.");
    }

    AutoBuilder.configureHolonomic(
      drive::getPose,
      drive::resetOdometry,
      drive::getRobotVelocity,
      drive::drive,
      new HolonomicPathFollowerConfig(
          new PIDConstants(5.0, 0.0, 0.0),
          new PIDConstants(5.0, 0.0, 0.0),
          // limit speeds in the paths, NOT HERE.
          Swerve.maxAttainableLinearSpeed,
          // Calculate drivetrain radius
          Math.sqrt(Math.pow(drive.swerveDriveConfiguration.moduleLocationsMeters[0].getY(), 2) + Math.pow(drive.swerveDriveConfiguration.moduleLocationsMeters[0].getX(), 2)),
          new ReplanningConfig() // customize this as desired
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );

    // swerve/maxSpeed is logged by YAGSL; it is the overall max speed, this only applies to teleop
    logger.addLoggable("swerve/teleop/Max Speed", this::getTeleopMaxLinearSpeed, true);
  }

  @Override
  public void periodic() {
    if (linearSpeedMultiplier.hasChanged(hashCode())) {
      drive.setMaximumSpeed(MathUtil.clamp(getMaxLinearSpeed(), 0, Swerve.maxAttainableLinearSpeed));
    }
    if (angularSpeedMultiplier.hasChanged(hashCode())) {
      drive.swerveController.setMaximumAngularVelocity(originalMaxAngularVelocity * angularSpeedMultiplier.get());
    }
    logger.logAll();
  }

  public double getMaxAngularSpeed() {
    return drive.getSwerveController().config.maxAngularVelocity;
  }

  public double getMaxLinearSpeed() {
    // calculate the value when called because we can't get it from YAGSL
    return Swerve.maxAttainableLinearSpeed * linearSpeedMultiplier.get();
  }

  /**
   * Calculate the robot's max linear speed during teleop based on the status
   * of the boost/precision buttons.
   * @return Max linear speed * overall boost/precision multiplier.
   */
  public double getTeleopMaxLinearSpeed() {
    final boolean boost = RobotContainer.driverController.rightBumper().getAsBoolean();
    final boolean precision = RobotContainer.driverController.leftBumper().getAsBoolean();
    // add boost/subtract precision according to button state
    final double multiplier = normalMultiplier.get() + (boost ? boostIncrease.get() : 0) - (precision ? precisionReduction.get() : 0);
    return getMaxLinearSpeed() * multiplier;
  }
}
