// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED.BlinkMode;
import frc.robot.subsystems.LED.Pattern;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.TalonFXUtil;
import frc.robot.utils.TunableNumber;

public class ShooterFlywheel extends SubsystemBase {
  public enum SpinType {
    disable,
    slowLeftMotor,
    slowRightMotor
  }

  public static final double maxRPM = 4700;

  /** Used to determine whether or not the shooter has reached the set speed. */
  public static final double rpmTolerance = 250.0;

  // 0.5 * m * r^2
  // mass = 0.1kg/wheel * 9 + 0.04kg/hex shaft * 3 + 0.1kg/gear * 3 + 0.04kg/pulley * 4
  public static final double flywheelMOI = 0.5 * 1.48 * 0.08;
  public static final double flywheelGearing = 1.0;

  private TunableNumber kS = new TunableNumber("Shooter/Flywheel/PIDF/kS", 0.0);
  private TunableNumber kV = new TunableNumber("Shooter/Flywheel/PIDF/kV", 0.14);
  private TunableNumber kP = new TunableNumber("Shooter/Flywheel/PIDF/kP", 0.5);
  private TunableNumber kI = new TunableNumber("Shooter/Flywheel/PIDF/kI", 0.0);
  private TunableNumber kD = new TunableNumber("Shooter/Flywheel/PIDF/kD", 0.0);
  
  // left/right are from perspective of robot (i.e. facing towards shooter)
  public final TalonFX leftMotor = new TalonFX(12);
  public final TalonFX rightMotor = new TalonFX(13);

  public final TunableNumber spinMultiplier = new TunableNumber("Shooter/Flywheel/Spin Multiplier (0-1)", 0.8);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

  public final FlywheelSim leftFlywheelSim = new FlywheelSim(
    DCMotor.getKrakenX60Foc(1),
    flywheelGearing,
    flywheelMOI
  );
  public final FlywheelSim rightFlywheelSim = new FlywheelSim(
    DCMotor.getKrakenX60Foc(1),
    flywheelGearing,
    flywheelMOI
  );
  private double lastLoopTime = Timer.getFPGATimestamp();

  private final BeaverLogger logger = new BeaverLogger();
  
  private Slot0Configs slot0Configs = new Slot0Configs();

  /** Creates a new ShooterFlywheel. */
  public ShooterFlywheel() {
    slot0Configs.kS = kS.get();
    slot0Configs.kV = kV.get();
    slot0Configs.kP = kP.get();
    slot0Configs.kI = kI.get();
    slot0Configs.kD = kD.get();

    leftMotor.getConfigurator().apply(slot0Configs);
    rightMotor.getConfigurator().apply(slot0Configs);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    leftMotor.setNeutralMode(NeutralModeValue.Coast);
    rightMotor.setNeutralMode(NeutralModeValue.Coast);

    TalonFXUtil.setCurrentLimit(leftMotor, 80);
    TalonFXUtil.setCurrentLimit(rightMotor, 80);

    logger.addLoggable("Shooter/Flywheel/Left RPM", () -> leftMotor.getRotorVelocity().getValueAsDouble() * 60, true);
    logger.addLoggable("Shooter/Flywheel/Right RPM", () -> rightMotor.getRotorVelocity().getValueAsDouble() * 60, true);
    logger.addLoggable("Shooter/Flywheel/Left Target RPM", () -> leftMotor.getClosedLoopReference().getValueAsDouble() * 60, true);
    logger.addLoggable("Shooter/Flywheel/Right Target RPM", () -> rightMotor.getClosedLoopReference().getValueAsDouble() * 60, true);
    logger.addLoggable("Shooter/Flywheel/Left Duty Cycle", () -> leftMotor.getDutyCycle().getValueAsDouble(), true);
    logger.addLoggable("Shooter/Flywheel/Right Duty Cycle", () -> rightMotor.getDutyCycle().getValueAsDouble(), true);
    logger.addLoggable("Shooter/Flywheel/Left Current (A)", () -> leftMotor.getSupplyCurrent().getValueAsDouble(), true);
    logger.addLoggable("Shooter/Flywheel/Right Current (A)", () -> leftMotor.getSupplyCurrent().getValueAsDouble(), true);
  }

  @Override
  public void periodic() {
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();
      leftMotor.getConfigurator().apply(slot0Configs);
      rightMotor.getConfigurator().apply(slot0Configs);
    }

    SmartDashboard.putBoolean("Shooter/Flywheel/At Set Speed", atSetSpeed());
    if (atSetSpeed()){
      RobotContainer.led.setSegmentPattern("flywheel", Pattern.INTAKE_NOTE, BlinkMode.SOLID);
    }else{
      RobotContainer.led.setSegmentPattern("flywheel", Pattern.RED, BlinkMode.SOLID);
    }
    logger.logAll();
  }

  @Override
  public void simulationPeriodic() {
    leftFlywheelSim.setInputVoltage(leftMotor.getSimState().getMotorVoltage());
    rightFlywheelSim.setInputVoltage(rightMotor.getSimState().getMotorVoltage());

    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastLoopTime;
    leftFlywheelSim.update(dt);
    rightFlywheelSim.update(dt);
    lastLoopTime = currentTime;

    leftMotor.getSimState().setRotorVelocity(leftFlywheelSim.getAngularVelocityRPM() / 60.0);
    rightMotor.getSimState().setRotorVelocity(rightFlywheelSim.getAngularVelocityRPM() / 60.0);
  }

  public boolean atSetSpeed() {
    // TODO: doesn't work in simulation, simulator never reaches >3000 RPM
    // error is in rps, convert to rpm
    return Math.abs(leftMotor.getClosedLoopError().getValueAsDouble() * 60) <= rpmTolerance
      && Math.abs(rightMotor.getClosedLoopError().getValueAsDouble() * 60) <= rpmTolerance;
  }

  /**
   * Set the speed of the flywheels. Remains at speed until called again.
   * @param rpm Flywheel RPM. Clamped to [0, maxRPM].
   * @param spinType Whether to use left or right motor to induce spin on note.
   */
  public void setSpeed(double rpm, SpinType spinType) {
    rpm = MathUtil.clamp(rpm, 0, maxRPM);
    double rps = rpm / 60.0;
    leftMotor.setControl(velocityRequest.withVelocity(
      spinType == SpinType.slowLeftMotor ? rps * spinMultiplier.get() : rps
    ));
    rightMotor.setControl(velocityRequest.withVelocity(
      spinType == SpinType.slowRightMotor ? rps * spinMultiplier.get() : rps
    ));
  }

  public void coast() {
    setMotors(0);
  }

  private void setMotors(double percentOutput) {
    percentOutput = MathUtil.clamp(percentOutput, -1, 1);
    leftMotor.set(percentOutput);
    rightMotor.set(percentOutput);
  }

  public Command getCoast() {
    return new InstantCommand(this::coast, this);
  }

  public Command getSetpointAxisControl(Supplier<Double> setpointAxis, SpinType spinType) {
    return new RunCommand(() -> setSpeed(MathUtil.clamp(setpointAxis.get() * maxRPM, 0, maxRPM), spinType), this);
  }

  public Command getPercentOutputControl(Supplier<Double> percentOutputAxis) {
    return new RunCommand(() -> setMotors(percentOutputAxis.get()), this);
  }
}
