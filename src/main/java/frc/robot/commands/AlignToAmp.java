// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTag;
import frc.robot.Constants.AprilTag.ID;
import frc.robot.RobotContainer;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.DriveUtil;

/**
 * Align intake with amp and drive towards it.
 * Speed is controlled by 'alignmentSpeedMultiplier'.
 * Driver can avoid obstacles while aligning to amp using left joystick.
 * <p>Automatically detects alliance colour.
 */
public class AlignToAmp extends Command {
  public static final double maxFieldX = 16.5;
  public static final double maxFieldY = 8.3;
  public static final double alignmentSpeedPGain = 5;
  private final Supplier<Double> alignmentSpeedMultiplier; // clamped to 0-1
  private ID aprilTagID;
  private Pose2d target = new Pose2d();
  private Transform2d offset = new Transform2d();
  private Rotation2d heading = new Rotation2d();
  private final BeaverLogger logger = new BeaverLogger();

  /**
   * Create a new AlignToAmp.
   * @param alignmentSpeedMultiplier A multiplier from 0-1 to apply to the translation speed.
   */
  public AlignToAmp(Supplier<Double> alignmentSpeedMultiplier) {
    this.alignmentSpeedMultiplier = alignmentSpeedMultiplier;
    addRequirements(RobotContainer.drive);
    logger.addLoggable("Commands/AlignToAmp/Estimated Time to Target (s)", this::getEstimatedTimeToTarget, true);
    logger.addLoggable("Commands/AlignToAmp/Translation Error (m)", this::getTranslationError, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // update alliance colour each time the command starts (e.g. if changed during testing)
    // also, if you do it in the constructor and the code starts before the DS or FMS connects,
    // it will grab the default (wrong) alliance
    aprilTagID = RobotContainer.isRedAlliance("AlignToAmp") ? ID.kRedAmp : ID.kBlueAmp;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target = AprilTag.getPose2d(aprilTagID);
    offset = target.minus(getPose());
    heading = target.getRotation().times(-1); // to face apriltag

    double[] driverAdjustment = DriveUtil.getDriverInputs(
      RobotContainer.driverController,
      true,
      false,
      false,
      false,
      DriveUtil.getSensitivity(RobotContainer.driverController),
      true
    );
    
    var teleopSpeedLimit = RobotContainer.drive.getLimitedTeleopLinearSpeed();
    var speed = clampAlignmentSpeedMultiplier() * teleopSpeedLimit * alignmentSpeedPGain;
    ChassisSpeeds speeds = RobotContainer.drive.drive.swerveController.getTargetSpeeds(
      // scale error from -1 to 1 so it is the same scale as driverAdjustment
      offset.getX() / maxFieldX + driverAdjustment[0],
      offset.getY() / maxFieldY + driverAdjustment[1],
      heading.getRadians(),
      getCurrentHeading().getRadians(),
      speed
    );

    speeds.vxMetersPerSecond = MathUtil.clamp(speeds.vxMetersPerSecond, -teleopSpeedLimit, teleopSpeedLimit);
    speeds.vyMetersPerSecond = MathUtil.clamp(speeds.vyMetersPerSecond, -teleopSpeedLimit, teleopSpeedLimit);

    RobotContainer.drive.setChassisSpeeds(speeds, true);

    SmartDashboard.putNumber("Commands/AlignToAmp/Alignment Speed Multiplier", speed);
    logger.logAll();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // adjust tolerances as required
    return Math.abs(getHeadingError()) < 2 && Math.abs(getXError()) < 0.05 && Math.abs(getYError()) < 0.05;
  }

  public double getEstimatedTimeToTarget() {
    // TODO: replace velocity calc with function in drive
    var velocity = RobotContainer.drive.drive.getRobotVelocity();
    var velocityNorm = Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
    return getTranslationError() / velocityNorm;
  }

  public double getTranslationError() {
    return offset.getTranslation().getNorm();
  }

  private double clampAlignmentSpeedMultiplier() {
    return MathUtil.clamp(alignmentSpeedMultiplier.get(), 0, 1);
  }

  private Pose2d getPose() {
    return new Pose2d(RobotContainer.drive.drive.getPose().getTranslation(), new Rotation2d())
      // TODO: measure actual distance
      .plus(new Transform2d(0, 0.45, new Rotation2d())); // offset distance from center of robot to front bumper
  }

  private Rotation2d getCurrentHeading() {
    return RobotContainer.drive.drive.getOdometryHeading();
  }

  private double getHeadingError() {
    return getCurrentHeading().getDegrees() - heading.getDegrees();
  }

  private double getXError() {
    return offset.getX();
  }

  private double getYError() {
    return offset.getY();
  }
}
