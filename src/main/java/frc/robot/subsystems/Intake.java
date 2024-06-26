// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.WaitForCounter;
import frc.robot.subsystems.LED.BlinkMode;
import frc.robot.subsystems.LED.Pattern;

public class Intake extends SubsystemBase {
  private final VictorSPX intakeMotor = new VictorSPX(14);
  private final PWMTalonSRX wideIntakeMotor = new PWMTalonSRX(0);
  private final DigitalInput intakeSensor = new DigitalInput(4);

  /**
   * Used by Visualizer to track the note since the intake sensor cannot see the note when the
   * elevator is raised.
   */
  public boolean noteHeld = false;

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(true);
    wideIntakeMotor.setInverted(false);

    SmartDashboard.putBoolean("Intake/Feeding", false);
    SmartDashboard.putBoolean("Intake/Intaking", false);
    SmartDashboard.putBoolean("Intake/Expelling", false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/Intake Sensor", getSensor());
    SmartDashboard.putBoolean("Intake/Note Held", noteHeld);
    if (getSensor()) { 
      // don't set it when false because we don't know if the note is still held (e.g. could be in elevator)
      noteHeld = true;
      RobotContainer.led.setDrive(Pattern.INTAKE_NOTE, BlinkMode.SOLID);
      RobotContainer.led.setHuman(Pattern.FIRE, BlinkMode.FIRE);
    } else if (Math.abs(intakeMotor.getMotorOutputPercent()) > 0.0) {
      RobotContainer.led.setDrive(Pattern.INTAKE_NO_NOTE, BlinkMode.BLINKING_ON);
      RobotContainer.led.setHuman(Pattern.FIRE, BlinkMode.FIRE);
    } else {
      RobotContainer.led.setDrive(Pattern.RED, BlinkMode.SOLID);
      RobotContainer.led.setHuman(Pattern.RED, BlinkMode.SOLID);
    }
  }

  /**
   * Get the status of the intake sensor.
   * Uses 'noteHeld' during simulation.
   */
  public boolean getSensor() {
    // intake sensor returns true when no note is present
    return RobotBase.isReal() ? !intakeSensor.get() : noteHeld;
  }

  /**
   * Set the intake motor. Runs motor at desired percent until called again.
   * @param percentOutput Output to run motor at between -1 and 1.
   */
  public void setMotor(double percentOutput) {
    SmartDashboard.putNumber("Intake/Motor Percent Output (-1-1)", percentOutput);
    intakeMotor.set(VictorSPXControlMode.PercentOutput, percentOutput);
    wideIntakeMotor.set(percentOutput);
  }

  // -- Command Factories

  /**
   * Expel the note (score on amp or trap), stopping after a timeout expires.
   * @return Command composition that expels a note.
   */
  public Command getExpelNote() {
    return getExpelNoteContinuously().withTimeout(0.2);
  }

  /**
   * Expel the note (score on amp or trap), stopping when interrupted.
   * @return Command composition that expels a note.
   */
  public Command getExpelNoteContinuously() {
    return Commands.startEnd(
      () -> {
        setMotor(-1);
        SmartDashboard.putBoolean("Intake/Expelling", true);
      },
      () -> { 
        setMotor(0);
        noteHeld = false;
        SmartDashboard.putBoolean("Intake/Expelling", false);
      },
      this
    );
  }

  /**
   * Feed the note to the shooter, stopping after a timeout expires.
   * @return Command composition that feeds a note to the shooter.
   */
  public ParallelRaceGroup getFeedNote() {
    return Commands.startEnd(
      () -> {
        setMotor(1);
        SmartDashboard.putBoolean("Intake/Feeding", true);
      },
      () -> { 
        setMotor(0);
        noteHeld = false;
        SmartDashboard.putBoolean("Intake/Feeding", false);
      },
      this
    ).withTimeout(1.0);
  }

  /**
   * Feed the note to the shooter once the shooter reaches its setpoint.
   * @return Command composition that waits until shooter is ready then feeds note.
   */
  public SequentialCommandGroup getFeedNoteOnReady() {
    return new WaitForCounter(
      () -> RobotContainer.shooterAngle.atTarget() && RobotContainer.shooterFlywheel.atSetSpeed(),
      3,
      "FeedNoteOnReady"
    ).andThen(getFeedNote());
  }

  /**
   * Run the intake until the sensor detects a note.
   * @return Command that intakes a note.
   */
  public Command getIntakeNote() {
    return Commands.startEnd(
      () -> { 
        setMotor(1.0);
        SmartDashboard.putBoolean("Intake/Intaking", true);
      },
      () -> {
        setMotor(0);
        SmartDashboard.putBoolean("Intake/Intaking", false);
      },
      this
    ).until(this::getSensor);
  }

  /**
   * Run the intake when the sensor doesn't detect a note.
   * @return Command that runs the intake a note whenever the sensor doesn't detect a note.
   */
  public Command getIntakeNoteContinuously() {
    return Commands.runEnd(
      () -> {
        boolean sensor = getSensor(); // use the same value for both lines below
        setMotor(sensor ? 0.0 : 1.0);
        SmartDashboard.putBoolean("Intake/Intaking", sensor);
      },
      () -> { 
        setMotor(0);
        SmartDashboard.putBoolean("Intake/Intaking", false);
      },
      this
    );
  }

  /**
   * Command that turns the intake motor off.
   * @return Command that turns the intake motor off.
   */
  public Command getTurnOff() {
    // super simple, but convenient if you want to use it in multiple places
    return new InstantCommand(() -> setMotor(0), this);
  }
}
