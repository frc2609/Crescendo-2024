// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simple arm feedforward to account for effects of gravity.
 * Send to SmartDashboard to tune constants.
 */
public class ArmFeedforward implements Sendable {
  public double kS;
  public double kG;
  public final double comDistanceFromPivot;
  public final double comAngleFromForwardDegrees;
  public final double mass;
  public final String ntTableName;

  /**
   * Create an ArmFeedForward.
   * @param kS Motor voltage required to overcome friction.
   * @param kG Motor voltage per Newton-metre of gravitational torque.
   * @param comDistanceFromPivot Distance between the center of mass and the arm pivot in metres.
   * @param mass Mass of arm in kg.
   * @param ntTableName Name to prefix SmartDashboard values with (e.g. "Shooter/Angle")
   */
  public ArmFeedforward(double kS, double kG, double comDistanceFromPivot, double comAngleFromForwardDegrees, double mass, String ntTableName) {
    this.kS = kS;
    this.kG = kG;
    this.comDistanceFromPivot = comDistanceFromPivot;
    this.comAngleFromForwardDegrees = comAngleFromForwardDegrees;
    this.mass = mass;
    this.ntTableName = ntTableName;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ArmFeedforward");
    builder.addDoubleProperty("kS", () -> { return kS; }, (double kS) -> { this.kS = kS; });
    builder.addDoubleProperty("kG", () -> { return kG; }, (double kG) -> { this.kG = kG; });
  }

  /**
   * Get the feedforward output.
   * @param angle Angle that increases as the arm moves away from the ground and where 0 = parallel to ground.
   * @return Motor voltage necessary to hold the arm up against gravity.
   */
  public double calculate(Rotation2d angle) {
    angle = angle.plus(Rotation2d.fromDegrees(comAngleFromForwardDegrees));
    double gravityPerpendicularToArm = -9.8 * mass * angle.getCos();
    double torque = gravityPerpendicularToArm * comDistanceFromPivot;
    SmartDashboard.putNumber(ntTableName + "/FF/Gravitational Torque (Nm)", torque);
    // voltage is opposite of gravitational torque (so the motor holds the arm up)
    double voltage = kS * Math.signum(-torque) + kG * -torque;
    MathUtil.clamp(voltage, -12, 12);
    SmartDashboard.putNumber(ntTableName + "/FF/Voltage Output (-12-12)", voltage);
    return voltage;
  }
}
