// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonSRX intakeMotor = new TalonSRX(14);
  // TODO: set actual DIO port
  private final DigitalInput intakeSensor = new DigitalInput(4);

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/Intake Sensor", getSensor());
  }

  /**
   * Get the status of the intake sensor.
   */
  public boolean getSensor() {
    // intake sensor returns true when no note is present
    return !intakeSensor.get();
  }

  /**
   * Set the intake motor. Runs motor at desired percent until called again.
   * @param percentOutput Output to run motor at between -1 and 1.
   */
  public void setMotor(double percentOutput) {
    SmartDashboard.putNumber("Intake/Motor Percent Output (-1-1)", percentOutput);
    intakeMotor.set(TalonSRXControlMode.PercentOutput, percentOutput);
  }
}
