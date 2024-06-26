// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.Swerve;
import frc.robot.commands.IdleShooter;
import frc.robot.commands.TeleopVelocityDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.LED.BlinkMode;
import frc.robot.subsystems.LED.Pattern;
import frc.robot.subsystems.ShooterFlywheel.SpinType;
import frc.robot.utils.TunableNumber;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    DataLogManager.start(); // log NetworkTables values
    DriverStation.startDataLog(DataLogManager.getLog()); // log DS data

    reportGitInfo();

    // PathPlanner logs its own info automatically (no need to here)
    
    robotContainer = new RobotContainer();
    RobotContainer.led.setDrive(Pattern.INTAKE_IDLE, BlinkMode.SOLID);
    RobotContainer.led.setHuman(Pattern.FIRE, BlinkMode.FIRE);

    // preload PathPlanner follow path command so it doesn't delay autonomous
    FollowPathCommand.warmupCommand().schedule();
    
    SmartDashboard.putNumber("Height Offset", 0.0);
    SmartDashboard.putNumber("Angle Offset", 2.0);
    SmartDashboard.putBoolean("Revert RPM change", false);
    
    SmartDashboard.putNumber("Angle Setpoint", 36.71);
    SmartDashboard.putNumber("RPM Setpoint", 4500.0);
    SmartDashboard.putBoolean("Spin Enabled", false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotContainer.visualizer.update();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.climber.stop();
    RobotContainer.elevator.stop();
    new IdleShooter().schedule(); // reset saved shooter setpoints on disable
  }

  @Override
  public void disabledPeriodic() {
    TunableNumber.updateAll();
  }

  @Override
  public void disabledExit() {
    RobotContainer.drive.drive.setMotorIdleMode(true);
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    RobotContainer.drive.drive.setMaximumSpeed(6.0);
    RobotContainer.drive.setDefaultCommand(new TeleopVelocityDrive(true));
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    RobotContainer.drive.drive.setMaximumSpeed(Swerve.maxModuleSpeed);
    RobotContainer.drive.removeDefaultCommand();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    
    SmartDashboard.putNumber("Test/Elevator Target Height (m)", Elevator.lowerLimitMeters);
    SmartDashboard.putNumber("Test/Shooter Target Angle (Deg)", ShooterAngle.reverseLimit.getDegrees());
    SmartDashboard.putNumber("Test/Shooter Target RPM", 0);

    // Elevator
    CommandScheduler.getInstance().schedule(
      new RunCommand(() -> RobotContainer.elevator.setHeight(SmartDashboard.getNumber("Test/Elevator Target Height (m)", 0)), RobotContainer.elevator)
    );

    // Shooter Angle
    CommandScheduler.getInstance().schedule(
      new RunCommand(() -> RobotContainer.shooterAngle.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("Test/Shooter Target Angle (Deg)", 0))), RobotContainer.shooterAngle)
    );

    // Shooter Flywheel
    CommandScheduler.getInstance().schedule(
      new RunCommand(() -> {
        SpinType spinType = SpinType.disable;
        if (RobotContainer.driverController.getHID().getLeftBumper())
          spinType = SpinType.slowLeftMotor;
        if (RobotContainer.driverController.getHID().getRightBumper())
          spinType = SpinType.slowRightMotor;
        RobotContainer.shooterFlywheel.setSpeed(SmartDashboard.getNumber("Test/Shooter Target RPM", 0), spinType);
      }, RobotContainer.shooterFlywheel)
    );
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * Report current branch and commit SHA to SmartDashboard. Cannot report whether any files have
   * been modified since the last commit.
   */
  private void reportGitInfo() {
    File deployDir = Filesystem.getDeployDirectory();
    File branchFile = new File(deployDir, "branch.txt");
    File commitFile = new File(deployDir, "commit.txt");

    try {
      SmartDashboard.putString("Git Branch", Files.readString(branchFile.toPath()));
    } catch (IOException e) {
      DataLogManager.log("Could not retrieve Git Branch.");
      e.printStackTrace();
    }

    try {
      SmartDashboard.putString("Git Commit SHA", Files.readString(commitFile.toPath()));
    } catch (IOException e) {
      DataLogManager.log("Could not retrieve Git Commit SHA.");
      e.printStackTrace();
    }
  }
}
