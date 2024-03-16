// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFlywheel.SpinType;

/**
 * Set the shooter to a desired preset angle, RPM, and spin.
 */
public class SetShooterToPreset extends Command {
  public enum ShooterPreset {
    kAtSpeaker(4000.0, 55.0, SpinType.slowRightMotor,
      new Pose2d(15.1, 5.5, Rotation2d.fromDegrees(0)),
      new Pose2d(1.4, 5.5, Rotation2d.fromDegrees(0))
    ),
    kAtPodium(4000.0, 35.0, SpinType.slowRightMotor,
      new Pose2d(13.8, 4.11, Rotation2d.fromDegrees(0)),
      new Pose2d(2.69, 4.11, Rotation2d.fromDegrees(0))
    ),
    kThrowNote(5800.0, 20.0, SpinType.slowRightMotor, new Pose2d(), new Pose2d());

    public final Rotation2d angle;
    public final double RPM;
    public final SpinType spinType;
    public final Pose2d redPose;
    public final Pose2d bluePose;

    private ShooterPreset(double RPM, double angleDeg, SpinType spinType, Pose2d redPose, Pose2d bluePose) {
      this.RPM = RPM;
      this.angle = Rotation2d.fromDegrees(angleDeg);
      this.spinType = spinType;
      this.redPose = redPose;
      this.bluePose = bluePose;
    }
  }

  private final ShooterPreset preset;
  private final boolean resetOdometry;

  /** Creates a new SetShooterToPreset. */
  public SetShooterToPreset(ShooterPreset preset, boolean resetOdometry) {
    this.preset = preset;
    this.resetOdometry = resetOdometry;
    addRequirements(RobotContainer.shooterAngle, RobotContainer.shooterFlywheel);
  }

  @Override
  public void initialize() {
    if (resetOdometry) {
      RobotContainer.drive.drive.resetOdometry(RobotContainer.isRedAlliance("SetShooterToPreset") ? preset.redPose : preset.bluePose);
    }
    RobotContainer.shooterAngle.setAngle(preset.angle);
    RobotContainer.shooterFlywheel.setSpeed(preset.RPM, preset.spinType);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterAngle.stop();
    RobotContainer.shooterFlywheel.coast();
  }
}
