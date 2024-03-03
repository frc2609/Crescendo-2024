// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Add your docs here. */
public class SimpleElevatorFeedforward implements Sendable {
  public double kS;
  public double kG;
  public double kV;
  public double massKg;

  public SimpleElevatorFeedforward(double kS, double kG, double kV, double massKg) {
    this.kS = kS;
    this.kG = kG;
    this.kV = kV;
    this.massKg = massKg;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ElevatorFeedforward");
    builder.addDoubleProperty("kS", () -> { return kS; }, (double kS) -> { this.kS = kS; });
    builder.addDoubleProperty("kG", () -> { return kG; }, (double kG) -> { this.kG = kG; });
    builder.addDoubleProperty("kV", () -> { return kV; }, (double kV) -> { this.kV = kV; });
  }

  /**
   * TODO: docs
   * @param velocity How fast the elevator is moving in m/s.
   * // no, velocity should have setpoint?
   * @return Motor power to hold elevator at current position.
   */
  public double calculate(double velocity) {
    return kS * Math.signum(velocity) + kG * massKg + kV * velocity;
  }
}
