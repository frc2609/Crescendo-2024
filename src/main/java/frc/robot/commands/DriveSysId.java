// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.RobotContainer;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;

/** Add your docs here. */
public class DriveSysId {
  // TODO: try different ramp rates and test lengths to see if results differ significantly

  public static SysIdRoutine angularVoltageSysIdRoutine(
      Config config, SubsystemBase swerveSubsystem, SwerveDrive swerveDrive, double maxVolts) {
    return new SysIdRoutine(
        config,
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> voltage) -> {
              // var modules = RobotContainer.drive.drive.getModules();
              // angle modules so robot spins counterclockwise
              // note that YAGSL angles only go from 0-180 (any higher or lower gets optimized...)
              // you have to set these by hand
              // modules[0].setAngle(135); // fl
              // modules[1].setAngle(45); // fr
              // modules[2].setAngle(225); // bl
              // modules[3].setAngle(315); // br

              SwerveDriveTest.powerDriveMotorsVoltage(
                  swerveDrive, Math.min(voltage.in(Volts), maxVolts));
            },
            log -> {
              for (SwerveModule module : swerveDrive.getModules()) {
                SwerveDriveTest.logDriveMotorVoltage(module, log);
              }
            },
            swerveSubsystem));
  }

  /**
   * Command to characterize the robot's response to heading changes using SysId
   * 
   * (probably)
   * 
   * <p>Mostly useful for estimating MOI of robot.
   * 
   * @return SysId Heading Command
   */
  public static Command sysIdHeadingCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        angularVoltageSysIdRoutine(
            new Config(),
            RobotContainer.drive, RobotContainer.drive.drive, 12),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public static Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            RobotContainer.drive, RobotContainer.drive.drive, 12),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public static Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            RobotContainer.drive, RobotContainer.drive.drive),
        3.0, 5.0, 3.0);
  }
}
