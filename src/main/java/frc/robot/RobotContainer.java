// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.AprilTagAmpAlign;
// import frc.robot.commands.AprilTagTrackDrive;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.ShootNote;
import frc.robot.commands.MoveElevatorToPosition.Position;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.utils.Visualizer;

public class RobotContainer {
  public static final Drive drive = new Drive(false);
  public static final Elevator elevator = new Elevator();
  public static final Intake intake = new Intake();
  public static final Limelight limelight = new Limelight();
  public static final ShooterAngle shooterAngle = new ShooterAngle();
  public static final ShooterFlywheel shooterFlywheel = new ShooterFlywheel();
  public static final Visualizer visualizer = new Visualizer();

  public static final CommandXboxController driverController = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand("IntakeNote", intake.getIntakeNote());
    NamedCommands.registerCommand("ShootNote", new ShootNote());
    NamedCommands.registerCommand("PrintOnCheckpoint", Commands.print("Reached Checkpoint!"));
    NamedCommands.registerCommand("WaitForButtonPress", Commands.waitUntil(driverController.back()));

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      drive.drive.field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      drive.drive.field.getObject("path").setPoses(poses);
    });

    // the auto specified here is chosen by default
    autoChooser = AutoBuilder.buildAutoChooser("Four Piece");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // TODO: Move these to 'Test' mode as applicable

    // Automation
    // TODO: leftBumper is already mapped to swerve precision mode
    driverController.leftBumper().onTrue(new ShootNote());

    // Swerve
    driverController.start().onTrue(new InstantCommand(drive.drive::zeroGyro));
    // driverController.y().whileTrue(AprilTagTrackDrive.getAlignToSpeaker());
    // driverController.a().whileTrue(new AprilTagAmpAlign());

    // Elevator
    driverController.povUp().onTrue(new MoveElevatorToPosition(Position.trap));
    driverController.povRight().onTrue(new MoveElevatorToPosition(Position.amp));
    driverController.povDown().onTrue(new MoveElevatorToPosition(Position.intake));

    // Intake
    // Fake the note being picked up during simulation.
    // Doesn't require intake so intake commands aren't cancelled when run.
    driverController.back().onTrue(new InstantCommand(() -> intake.noteHeld = true));
    driverController.a().onTrue(intake.getIntakeNote());
    driverController.b().onTrue(intake.getExpelNote());
    driverController.y().onTrue(intake.getFeedNote());
    driverController.x().onTrue(intake.getTurnOff());
    
    // Shooter Angle
    
    // Shooter Flywheel
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Convenience function to check the current alliance according to the DS or FMS (if connected).
   * Assumes blue alliance if alliance is invalid.
   * @param callerName Name of code that depends on result. Used to report an error if the alliance isn't detected.
   * @return Whether robot is on red alliance.
   */
  public static boolean isRedAlliance(String callerName) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    } else {
      System.out.println("No alliance detected for " + callerName + ": Assuming blue alliance.");
      return false;
    }
  }
}
