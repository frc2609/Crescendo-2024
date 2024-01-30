// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TeleopVelocityDrive;
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
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

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
    RobotContainer.drive.setDefaultCommand(new TeleopVelocityDrive(true));
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    RobotContainer.drive.removeDefaultCommand();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    robotPeriodic();
  }

  /**
   * Report current branch and commit SHA to SmartDashboard. Cannot report
   * whether any files have been modified since the last commit.
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
