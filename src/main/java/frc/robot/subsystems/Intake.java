// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final VictorSPX intakeMotor = new VictorSPX(14);
  // TODO: set actual DIO port
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
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/Intake Sensor", getSensor());
    SmartDashboard.putBoolean("Intake/Note Held", noteHeld);
    // don't set it when false because we don't know if the note is still held
    if (getSensor()) noteHeld = true;
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
  }

  /**
   * Run the intake until the sensor detects a note.
   * @return Command that intakes a note.
   */
  public Command getIntakeNote() {
    return Commands.startEnd(
      () -> setMotor(0.7),
      () -> setMotor(0),
      this
    ).until(this::getSensor);
  }

  /**
   * Expel the note (score on amp or trap), stopping after a timeout expires.
   * @return Command composition that expels a note.
   */
  public ParallelRaceGroup getExpelNote() {
    return Commands.startEnd(
      () -> setMotor(-1),
      () -> { 
        setMotor(0);
        noteHeld = false;
      },
      this
    ).withTimeout(0.2);
  }

  /**
   * Feed the note to the shooter, stopping after a timeout expires.
   * @return Command composition that feeds a note to the shooter.
   */
  public ParallelRaceGroup getFeedNote() {
    return Commands.startEnd(
      () -> setMotor(1),
      () -> { 
        setMotor(0);
        noteHeld = false;
      },
      this
    ).withTimeout(0.4);
  }

  /**
   * Command that turns the intake motor off.
   * @return Command that turns the intake motor off.
   */
  public Command getTurnOff() {
    // super simple, but convenient if you want to use it in multiple places
    return new InstantCommand(() -> setMotor(0));
  }
}
