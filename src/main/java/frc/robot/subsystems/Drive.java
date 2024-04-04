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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AprilTag;
import frc.robot.Constants.Swerve;
import frc.robot.utils.BeaverLogger;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Drive extends SubsystemBase {
  private final Rotation2d headingTolerance = Rotation2d.fromDegrees(4);
  private final double originalMaxAngularSpeed;

  /** All YAGSL functionality is implemented in this instance of SwerveDrive. */
  public final SwerveDrive drive;
  private Optional<Rotation2d> headingOverride = Optional.empty();
  private Optional<ChassisSpeeds> targetRobotRelativeSpeeds = Optional.empty();
  private final BeaverLogger logger = new BeaverLogger();

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
      this::getPoseEfficiently,
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
          drive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
          new ReplanningConfig()
      ),
      () -> {
        return RobotContainer.isRedAlliance("Drive (PathPlanner)");
      },
      this
    );

    SmartDashboard.putNumber("swerve/teleop/linearSpeedMultiplier", 1.33);
    SmartDashboard.putNumber("swerve/teleop/angularSpeedMultiplier", 1.0);
    logger.addLoggable("swerve/Overall Speed (mps)", this::getVelocity, true);
    logger.addLoggable("swerve/Target Heading (deg)", () -> Math.toDegrees(RobotContainer.drive.drive.swerveController.lastAngleScalar), true);
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
    
    if (targetRobotRelativeSpeeds.isPresent()) {
      applyChassisSpeeds();
    } else if (headingOverride.isPresent()) {
      targetRobotRelativeSpeeds = Optional.of(new ChassisSpeeds());
      applyChassisSpeeds();
    }
    
    SmartDashboard.putBoolean("swerve/Odometry Out Of Range", odometryOutOfRange());
    logger.logAll();
  }

  /**
   * Set the gyro to 0 degrees when the robot is facing forward, relative to your alliance colour.
   * (i.e. Pointing the robot forward will always reset the gyro correctly no matter which alliance you are on.)
   * <p>Note that 'SwerveDrive.resetGyro' DOES NOT reset the gyro according to alliance colour, so
   * you have to point the robot forward relative to the blue alliance.
   */
  public void teleopResetGyro() {
    drive.resetOdometry(new Pose2d(getPoseEfficiently().getTranslation(), Rotation2d.fromDegrees(RobotContainer.isRedAlliance("Drive::ZeroGyro()") ? 180 : 0)));
  }

  /**
   * Check if odometry exceeds the border of the field.
   * @return If odometry exceeds the border of the field.
   */
  public boolean odometryOutOfRange() {
    var pose = getPoseEfficiently();
    return pose.getX() < 0 || pose.getX() > AprilTag.fieldLayout.getFieldLength()
      || pose.getY() < 0 || pose.getY() > AprilTag.fieldLayout.getFieldWidth();
  }

  /**
   * Get the cached robot pose from SwerveDrive's Field2d.
   * <p>Significantly faster than {@code drive.getPose()} since this does not use the pose estimator
   * directly, so code doesn't have to wait for {@code drive.odometryLock}.
   * @return The pose of the robot.
   */
  public Pose2d getPoseEfficiently() {
    /*
     * The robot pose on the field is updated at the same time as odometry, so getting the robot
     * pose in this way means the code doesn't have to wait on 'drive.odometryLock'.
     * HOWEVER, it doesn't update when telemetry verbosity is below 'low' (so don't do that).
     */
    return drive.field.getRobotPose();
  }

  /**
   * Get the overall velocity (combined x and y) of the robot.
   * @return Velocity of the robot in m/s.
   */
  public double getVelocity() {
    var speeds = drive.getRobotVelocity();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * Align to the provided heading by overriding the rotation velocity of the ChassisSpeeds
   * provided to {@link #setChassisSpeeds(ChassisSpeeds, boolean) setChassisSpeeds()}.
   * <p>Must be called each loop cycle to function. Stops overriding heading when no longer called.
   * <p>If {@link #setChassisSpeeds(ChassisSpeeds, boolean) setChassisSpeeds()} is not called, will
   * rotate in place.
   * @param headingOverride The heading to align the robot to.
   */
  public void overrideHeading(Rotation2d headingOverride) {
    this.headingOverride = Optional.of(headingOverride);
  }

  /**
   * Set the desired ChassisSpeeds for swerve drive.
   * <p>This should be the only drive function you call. (YAGSL's functions bypass {@link
   * #overrideHeading(Rotation2d) overrideHeading()}.)
   * @param chassisSpeeds Desired ChassisSpeeds.
   * @param isFieldRelative Whether to use 'chassisSpeeds' as field relative speeds or robot relative speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean isFieldRelative) {
    targetRobotRelativeSpeeds = Optional.of(
      isFieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, drive.getOdometryHeading())
      : chassisSpeeds
    );
  }

  /**
   * Tell YAGSL to drive at the desired ChassisSpeeds, overriding heading as applicable.
   */
  private void applyChassisSpeeds() {
    if (headingOverride.isPresent()) {
      SmartDashboard.putNumber("swerve/Heading Override (deg)", headingOverride.get().getDegrees());
      // calculate speed according to heading override
      targetRobotRelativeSpeeds.get().omegaRadiansPerSecond = drive.swerveController.headingCalculate(
        drive.getOdometryHeading().getRadians(),
        headingOverride.get().getRadians()
      );
      // empty the optional so heading control is returned unless the heading is overridden in the next loop cycle
      headingOverride = Optional.empty();
    }

    drive.drive(targetRobotRelativeSpeeds.get());
    // empty the optional so the robot stops driving in the next loop cycle unless chassisSpeeds are set again
    targetRobotRelativeSpeeds = Optional.empty();
  }
}
