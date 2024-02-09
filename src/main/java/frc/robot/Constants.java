// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import swervelib.math.SwerveMath;

/**
 * Robot-wide constants should go here.
 * <p>If a constant only pertains to a single part of the robot (e.g. a motor
 * CAN ID) or a specific command (e.g. kP for autobalance), put it in that 
 * file—not here!
 */
public final class Constants {
  /** Swerve drive related constants. */
  public final static class Swerve {
    /** The maximum possible RPM of a REV NEO v1.0/v1.1 motor. */
    public static final double maxNEORPM = 5676;
    /** Make sure to adjust this as the wheels wear. */
    public static final double wheelDiameter = Units.inchesToMeters(4);
    // 5 : 1 drives 4 : 1 ultraplanetary attached to 25T pulley spinning 75T module pulley
    // divided by 1 because YAGSL divides by gear ratio
    public static final double angleGearRatio = 1.0 / ((1.0 / 3.0) * UltraPlanetaryRatios.fiveToOne * UltraPlanetaryRatios.fourToOne);
    // 12 tooth gear drives 32 tooth gear attached to 3 : 1 bevel gears
    // divided by 1 because YAGSL divides by gear ratio
    public static final double driveGearRatio = 1.0 / ((12.0 / 32.0) * (1.0 / 3.0));
    /** Converts rotations -> meters per rotation. */
    public static final double driveConversionFactor = SwerveMath.calculateMetersPerRotation(wheelDiameter, driveGearRatio, 1);
    /** Converts rotations -> degrees per rotation. */
    public static final double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(angleGearRatio, 1);
    /** The maximum linear speed a swerve module can achieve in m/s. */
    public static final double maxModuleSpeed = (maxNEORPM / 60.0) * driveConversionFactor;
  }

  /**
   * UltraPlanetary gearbox ratios differ from the ratio printed on
   * the gearbox's side. This class contains the actual ratios of the
   * UltraPlanetary cartridges.
   */
  public static final class UltraPlanetaryRatios {
    /** The ratio of a 3:1 UltraPlanetary cartridge used as a reduction. */
    public static final double threeToOne = 1.0 / (84.0 / 29.0);
    /** The ratio of a 4:1 UltraPlanetary cartridge used as a reduction. */
    public static final double fourToOne = 1.0 / (76.0 / 21.0);
    /** The ratio of a 5:1 UltraPlanetary cartridge used as a reduction. */
    public static final double fiveToOne = 1.0 / (68.0 / 13.0);
  }

  /** 
   * Xbox controller related constants. 
   * Do not put button or axis numbers in here, instead use the functions it
   * provides, such as getLeftY() or a() / b() / x() / y().
   */
  public final static class Xbox {
    /** 
     * Not the same as the value in the YAGSL configuration!
     * <p>Applies to translation and rotation VELOCITY.
     * The value in the YAGSL configuration is automatically applied to the
     * rotation joystick when controlling the robot with a desired angle.
     */
    public static final double joystickDeadband = 0.075;
  }
}
