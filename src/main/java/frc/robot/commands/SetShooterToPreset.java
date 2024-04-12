// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFlywheel.SpinType;

/**
 * Set the shooter to a desired preset angle, RPM, and spin.
 */
public class SetShooterToPreset extends Command {
  public enum ShooterPreset {
    kAtSpeaker(4000.0, 61.0, SpinType.slowRightMotor,
      new Translation2d(15.1, 5.5),
      new Translation2d(1.4, 5.5)
    ),
    kAtPodium(4000.0, 40.0, SpinType.slowRightMotor,
      new Translation2d(13.8, 4.11),
      new Translation2d(2.69, 4.11)
    ),
    kThrowNoteLow(3500.0, 20.0, SpinType.disable, new Translation2d(), new Translation2d()),
    kThrowNoteHigh(3500.0, 45.0, SpinType.disable, new Translation2d(), new Translation2d());

    public final Rotation2d angle;
    public final double RPM;
    public final SpinType spinType;
    public final Translation2d redTranslation;
    public final Translation2d blueTranslation;

    private ShooterPreset(double RPM, double angleDeg, SpinType spinType, Translation2d redTranslation, Translation2d blueTranslation) {
      this.RPM = RPM;
      this.angle = Rotation2d.fromDegrees(angleDeg);
      this.spinType = spinType;
      this.redTranslation = redTranslation;
      this.blueTranslation = blueTranslation;
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
      var translation = RobotContainer.isRedAlliance("SetShooterToPreset") ? preset.redTranslation : preset.blueTranslation;
      RobotContainer.drive.drive.resetOdometry(new Pose2d(translation, RobotContainer.drive.drive.getOdometryHeading()));
    }
  }

  @Override
  public void execute() {
    RobotContainer.shooterAngle.setAngle(preset.angle);
    RobotContainer.shooterFlywheel.setSpeed(preset.RPM, preset.spinType);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterAngle.stop();
    RobotContainer.shooterFlywheel.coast();
  }
}
