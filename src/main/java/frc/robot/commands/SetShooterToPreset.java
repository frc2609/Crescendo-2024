// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterFlywheel.SpinType;

/**
 * Set the shooter to a desired preset angle, RPM, and spin.
 */
public class SetShooterToPreset extends Command {
  public enum ShooterPreset {
    // TODO: placeholders; tune
    kAtSpeaker(3000.0, 55.0, SpinType.disable),
    kAtPodium(4000.0, 40.0, SpinType.disable),
    kThrowNote(5800.0, 55.0, SpinType.slowLeftMotor);

    public final Rotation2d angle;
    public final double RPM;
    public final SpinType spinType;

    private ShooterPreset(double RPM, double angleDeg, SpinType spinType) {
      this.RPM = RPM;
      this.angle = Rotation2d.fromDegrees(angleDeg);
      this.spinType = spinType;
    }
  }

  private final ShooterPreset preset;

  /** Creates a new SetShooterToPreset. */
  public SetShooterToPreset(ShooterPreset preset) {
    this.preset = preset;
    addRequirements(RobotContainer.shooterAngle, RobotContainer.shooterFlywheel);
  }

  @Override
  public void initialize() {
    RobotContainer.shooterAngle.setAngle(preset.angle);
    RobotContainer.shooterFlywheel.setSpeed(preset.RPM, preset.spinType);
  }
}
