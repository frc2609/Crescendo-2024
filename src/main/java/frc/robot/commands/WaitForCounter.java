// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitForCounter extends Command {
  private final double maxCounter;
  private double counter = 0;
  private final Supplier<Boolean> condition;
  private final String callerName;

  /**
   * Wait until a condition has been true for >= 'maxCounter' consecutive loop cycles.
   * @param condition Condition to check.
   * @param maxCounter Amount of consecutive loop cycles 'condition' must be true for.
   * @param callerName Name of code waiting for counter (to display counter on SmartDashboard).
   */
  public WaitForCounter(BooleanSupplier condition, int maxCounter, String callerName) {
    // this constructor is useful for Triggers, as they extend 'BooleanSupplier'
    this.condition = condition::getAsBoolean;
    this.maxCounter = maxCounter;
    this.callerName = callerName;
  }

  /**
   * Wait until a condition has been true for >= 'maxCounter' consecutive loop cycles.
   * @param condition Condition to check.
   * @param maxCounter Amount of consecutive loop cycles 'condition' must be true for.
   * @param callerName Name of code waiting for counter (to display counter on SmartDashboard).
   */
  public WaitForCounter(Supplier<Boolean> condition, int maxCounter, String callerName) {
    this.condition = condition;
    this.maxCounter = maxCounter;
    this.callerName = callerName;
  }

  @Override
  public void initialize() {
    counter = 0;
  }

  @Override
  public void execute() {
    if (condition.get()) {
      counter++;
    } else {
      counter = 0;
    }
    SmartDashboard.putNumber(callerName + "/Counter", counter);
  }

  @Override
  public boolean isFinished() {
    return counter >= maxCounter;
  }
}
