// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class NetworkPushButton {
  /**
   * Put a button (boolean value) on NetworkTables, then runs a function when pressed (true).
   * <p>Resets boolean when pressed so function does not run until button is pressed again.
   * <p>Button works when robot is disabled.
   * <p>Also see {@link edu.wpi.first.wpilibj2.command.button.NetworkButton NetworkButton}.
   * Similar, but it does not reset button when pressed (acts like a toggle).
   * @param buttonName Name of the button, e.g. "swerve/Reset Odometry to Limelight"
   * @param runOnTrue Function to run when button is true.
   * @param putInButtonsTable Put this button in "Buttons/" table when true.
   */
  public NetworkPushButton(
    String buttonName,
    Runnable runOnTrue,
    boolean putInButtonsTable
  ) {
    var name = putInButtonsTable ? "Buttons/" + buttonName : buttonName;
    SmartDashboard.putBoolean(name, false);
    new Trigger(() -> SmartDashboard.getBoolean(name, false))
      .onTrue(
        new InstantCommand(runOnTrue).ignoringDisable(true)
        .andThen(new InstantCommand(() -> SmartDashboard.putBoolean(name, false)).ignoringDisable(true))
      );
  }
}
