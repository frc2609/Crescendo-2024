// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.AutoScoreAmp;
import frc.robot.commands.IdleShooter;
import frc.robot.commands.MoveElevatorToPosition;
import frc.robot.commands.PassthroughNote;
// import frc.robot.commands.ResetIntakeAndElevator;
import frc.robot.commands.MoveElevatorToPosition.Position;
import frc.robot.commands.SetShooterToPreset.ShooterPreset;
import frc.robot.commands.ResetIntakeAndElevator;
import frc.robot.commands.SetShooterToPreset;
import frc.robot.commands.SetShooterToDashboard;
import frc.robot.commands.ShootNote;
import frc.robot.commands.ShootNoteContinuously;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterAngle;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.Limelight.Pipeline;
import frc.robot.subsystems.ShooterFlywheel.SpinType;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.NetworkPushButton;
import frc.robot.utils.Visualizer;

public class RobotContainer {
  public static final CommandXboxController driverController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);

  public static final Climber climber = new Climber();
  public static final Drive drive = new Drive(false);
  public static final Elevator elevator = new Elevator();
  public static final Intake intake = new Intake();
  public static final Limelight rearLimelight = new Limelight("limelight-shooter");
  public static final ShooterAngle shooterAngle = new ShooterAngle();
  public static final ShooterFlywheel shooterFlywheel = new ShooterFlywheel();
  public static final Vision vision = new Vision();
  public static final Visualizer visualizer = new Visualizer();
  public static final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  public static final LED led = new LED();

  private final BeaverLogger logger = new BeaverLogger();
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand("ResetPoseToLimelight", rearLimelight.getResetRobotPose());
    NamedCommands.registerCommand("IntakeNote", intake.getIntakeNote());
    NamedCommands.registerCommand("IntakeNoteContinuously", RobotContainer.intake.getIntakeNoteContinuously());
    NamedCommands.registerCommand("ShootNote", new ShootNote(SpinType.disable));
    NamedCommands.registerCommand("ShootNoteContinuously", new ShootNoteContinuously(SpinType.disable));
    NamedCommands.registerCommand("PrintOnCheckpoint", Commands.print("Reached Checkpoint!"));
    NamedCommands.registerCommand("TimedDriveForward", new RunCommand(() -> drive.setChassisSpeeds(new ChassisSpeeds(isRedAlliance("TimedDriveForward") ? -1 : 1, 0, 0), true), drive).withTimeout(2));
    NamedCommands.registerCommand("WaitForButtonPress", Commands.waitUntil(driverController.back()));
    NamedCommands.registerCommand("SwitchTo2DPipeline", new InstantCommand(() -> rearLimelight.setPipeline(Pipeline.track2d)));
    NamedCommands.registerCommand("PassthroughNote", new PassthroughNote());

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      drive.drive.field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      drive.drive.field.getObject("path").setPoses(poses);
    });

    // the auto specified here is chosen by default
    autoChooser = AutoBuilder.buildAutoChooser("Four Piece");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    logger.addLoggable("General/Total Current (A)", pdh::getTotalCurrent, true);
  }

  private void configureBindings() {
    // Automation
    driverController.leftBumper().toggleOnTrue(
      new ShootNote(SpinType.slowRightMotor).handleInterrupt(() -> new IdleShooter().schedule())
    );
    driverController.rightBumper().onTrue(
      new SequentialCommandGroup(
        new MoveElevatorToPosition(Position.amp),
        new WaitUntilCommand(() -> !driverController.getHID().getRightBumper()),
        RobotContainer.intake.getExpelNote(),
        Commands.waitSeconds(0.25),
        new ResetIntakeAndElevator()
      )
    );

    // Climber
    driverController.povUp()
      .whileTrue(climber.getMove(() -> 0.6))
      .onFalse(climber.getHold());
    driverController.povDown()
      .whileTrue(climber.getMove(() -> -0.8))
      .onFalse(climber.getHold());
    new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.1)
      .whileTrue(climber.getMove(operatorController::getLeftTriggerAxis))
      .onFalse(climber.getHold());
      new Trigger(() -> operatorController.getRightTriggerAxis() > 0.1)
      .whileTrue(climber.getMove(() -> -operatorController.getRightTriggerAxis()))
      .onFalse(climber.getHold());
    
    // Drive
    driverController.start().onTrue(new InstantCommand(drive::teleopResetGyro).ignoringDisable(true));
    
    // Elevator
    driverController.povRight().onTrue(new MoveElevatorToPosition(Position.amp));
    driverController.povLeft().onTrue(new MoveElevatorToPosition(Position.intake));
    operatorController.povUp().onTrue(new MoveElevatorToPosition(Position.amp));
    operatorController.povDown().onTrue(new MoveElevatorToPosition(Position.intake));

    // Intake
    // Fake the note being picked up during simulation.
    new NetworkPushButton("Intake NoteHeld Override", () -> intake.noteHeld = true, true);
    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.2).whileTrue(intake.getExpelNoteContinuously());
    new Trigger(() -> driverController.getRightTriggerAxis() > 0.2).whileTrue(intake.getIntakeNoteContinuously());
    operatorController.a().toggleOnTrue(intake.getIntakeNote());
    operatorController.b().onTrue(intake.getExpelNote());
    operatorController.y().onTrue(intake.getFeedNoteOnReady());
    operatorController.x().onTrue(intake.getTurnOff());
    operatorController.back().onTrue(intake.getFeedNote());

    // Shooter
    // operatorController.leftBumper().whileTrue(new SetShooterToPreset(ShooterPreset.kAtSpeaker, false));
    operatorController.leftBumper().whileTrue(new SetShooterToDashboard());
    operatorController.rightBumper().whileTrue(new SetShooterToPreset(ShooterPreset.kAtPodium, true));
    operatorController.povLeft().whileTrue(new SetShooterToPreset(ShooterPreset.kThrowNoteLow, false));
    operatorController.povRight().whileTrue(new SetShooterToPreset(ShooterPreset.kThrowNoteHigh, false));

    // Vision
    operatorController.start().onTrue(rearLimelight.getResetRobotPose());
    operatorController.leftStick().whileTrue(rearLimelight.getEstimateRobotPose());
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
