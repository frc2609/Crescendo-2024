// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class TalonFXUtil {
  public static void setCurrentLimit(TalonFX motor, double currentLimit) {
    var configurator = motor.getConfigurator();
    var limits = new TalonFXConfiguration().CurrentLimits;
    configurator.refresh(limits); // get current settings from motor
    // write only the settings we want to change
    configurator.apply(
      // TODO: what is the difference between supply and stator current limits?
      limits.withStatorCurrentLimit(currentLimit)
        .withStatorCurrentLimitEnable(true)
    );
  }
}
