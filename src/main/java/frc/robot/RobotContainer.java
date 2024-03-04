// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AprilTagTrackDrive;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.MoveElevatorToPosition.Position;
import frc.robot.Constants.AprilTag.ID;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterFlywheel.SpinType;
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
    SmartDashboard.putNumber("shooter setpoint", 0);

    configureBindings();

    NamedCommands.registerCommand("printOnCheckpoint", Commands.print("Reached Checkpoint!"));
    NamedCommands.registerCommand("Autobalance", Commands.print("Autobalancing!"));
    NamedCommands.registerCommand("ScorePiece1", Commands.print("Scoring Piece 1!"));
    NamedCommands.registerCommand("PickupPiece2", Commands.print("Picking Up Piece 2!"));
    NamedCommands.registerCommand("ScorePiece2", Commands.print("Scoring Piece 2!"));
    NamedCommands.registerCommand("WaitForButtonPress", Commands.waitUntil(driverController.a()::getAsBoolean));

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      drive.drive.field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      drive.drive.field.getObject("path").setPoses(poses);
    });

    // the auto specified here is chosen by default
    autoChooser = AutoBuilder.buildAutoChooser("Two Piece & Balance");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Swerve
    driverController.x().onTrue(new InstantCommand(drive.drive::lockPose));
    driverController.start().onTrue(new InstantCommand(drive.drive::zeroGyro));
    driverController.y().whileTrue(new AprilTagTrackDrive(true, ID.kRedSpeakerCenter));
    
    // Elevator
    driverController.povUp().onTrue(new MoveElevatorToPosition(Position.trap));
    driverController.povRight().onTrue(new MoveElevatorToPosition(Position.amp));
    driverController.povDown().onTrue(new MoveElevatorToPosition(Position.intake));
    // elevator.setDefaultCommand(new RunCommand(() -> elevator.setHeight(driverController.getLeftTriggerAxis()), elevator));

    // Intake
    // Fake the note being picked up during simulation.
    // Doesn't require intake so intake commands aren't cancelled when run.
    driverController.back().onTrue(new InstantCommand(() -> intake.noteHeld = true));
    // TODO: add some intake commands
    
    // Shooter Angle
    // driverController.a().onTrue(new InstantCommand(() -> {
    //   shooterAngle.setAngle(Rotation2d.fromDegrees(0));
    // }, shooterAngle));
    // driverController.b().onTrue(new InstantCommand(() -> {
    //   shooterAngle.setAngle(Rotation2d.fromDegrees(25));
    // }, shooterAngle));
    // driverController.y().onTrue(new InstantCommand(() -> {
    //   shooterAngle.setAngle(Rotation2d.fromDegrees(55));
    // }, shooterAngle));
    // driverController.x().onTrue(new InstantCommand(() -> {
    //   shooterAngle.setAngle(Rotation2d.fromDegrees(75));
    // }, shooterAngle));
    // shooterAngle.setDefaultCommand(new RunCommand(() -> shooterAngle.setAngle(Rotation2d.fromDegrees(driverController.getRightTriggerAxis() * 60.0)), shooterAngle));
    
    // Shooter Flywheel
    shooterFlywheel.setDefaultCommand(new RunCommand(() -> {
      SpinType spinType = SpinType.disable;
      // if (driverController.leftBumper().getAsBoolean())
        spinType = SpinType.slowLeftMotor;
      // if (driverController.rightBumper().getAsBoolean())
      //   spinType = SpinType.slowRightMotor;
      shooterFlywheel.setSpeed(SmartDashboard.getNumber("shooter setpoint", 0), spinType);
    }, shooterFlywheel));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
