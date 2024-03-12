// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.IdleShooter;
import frc.robot.commands.TeleopVelocityDrive;
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
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    RobotContainer.visualizer.update();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
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
    RobotContainer.drive.drive.setHeadingCorrection(false);
    RobotContainer.drive.setDefaultCommand(new TeleopVelocityDrive(true));
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    // avoid interfering with PathPlanner's heading PIDs
    RobotContainer.drive.drive.setHeadingCorrection(false);
    RobotContainer.drive.removeDefaultCommand();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    
    SmartDashboard.putNumber("Test/Elevator Target Height (m)", 0);
    SmartDashboard.putNumber("Test/Shooter Target Angle (Deg)", 0);
    SmartDashboard.putNumber("Test/Shooter Target RPM", 0);

    // Elevator
    CommandScheduler.getInstance().schedule(
      new RunCommand(() -> RobotContainer.elevator.setHeight(SmartDashboard.getNumber("Test/Elevator Target Height (m)", 0)), RobotContainer.elevator)
    );

    // Shooter Angle
    // CommandScheduler.getInstance().schedule(
    //   new RunCommand(() -> RobotContainer.shooterAngle2.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("Test/Shooter Target Angle (Deg)", 0))), RobotContainer.shooterAngle2)
    // );

    // Shooter Flywheel
    CommandScheduler.getInstance().schedule(
      new RunCommand(() -> {
        SpinType spinType = SpinType.disable;
        if (RobotContainer.driverController.leftBumper().getAsBoolean())
          spinType = SpinType.slowLeftMotor;
        if (RobotContainer.driverController.rightBumper().getAsBoolean())
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
