// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.BeaverLogger;
import frc.robot.utils.TunableNumber;

public class ShooterFlywheel extends SubsystemBase {
  public enum SpinType {
    disable,
    slowLeftMotor,
    slowRightMotor
  }

  /** Used to determine whether or not the shooter has reached the set speed. */
  public static final double rpmTolerance = 100.0;

  // 0.5 * m * r^2
  // mass = 0.1kg/wheel * 6 + 0.04kg/hex shaft * 3 + 0.1kg/gear * 2 + 0.04kg/pulley * 2 + 0.05/metal pulley * 2
  public static final double flywheelMOI = 0.5 * 1.1 * 0.08;
  public static final double flywheelGearing = 1.0;

  private TunableNumber kS = new TunableNumber("kS", 0.0);
  private TunableNumber kV = new TunableNumber("kV", 0.5);
  private TunableNumber kP = new TunableNumber("kP", 0.0);
  private TunableNumber kI = new TunableNumber("kI", 0.0);
  private TunableNumber kD = new TunableNumber("kD", 0.0);
  

  // left/right from perspective of shooter (i.e. pointing towards back of robot)
  public final TalonFX leftMotor = new TalonFX(12);
  public final TalonFX rightMotor = new TalonFX(13);

  public final TunableNumber spinMultiplier = new TunableNumber("Shooter/Flywheel/Spin Multiplier (0-1)", 0.8);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

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
    slot0Configs.kS = 0.0;
    slot0Configs.kV = 0.5;
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    leftMotor.getConfigurator().apply(slot0Configs);
    rightMotor.getConfigurator().apply(slot0Configs);

    leftMotor.setNeutralMode(NeutralModeValue.Coast);
    rightMotor.setNeutralMode(NeutralModeValue.Coast);

    logger.addLoggable("Shooter/Flywheel/Left RPM", () -> leftMotor.getRotorVelocity().getValueAsDouble() * 60, true);
    logger.addLoggable("Shooter/Flywheel/Right RPM", () -> rightMotor.getRotorVelocity().getValueAsDouble() * 60, true);
    logger.addLoggable("Shooter/Flywheel/Left Target RPM", () -> leftMotor.getClosedLoopReference().getValueAsDouble() * 60, true);
    logger.addLoggable("Shooter/Flywheel/Right Target RPM", () -> rightMotor.getClosedLoopReference().getValueAsDouble() * 60, true);
  }

  @Override
  public void periodic() {
    logger.logAll();
    SmartDashboard.putBoolean("Shooter/Flywheel/At Set Speed", atSetSpeed());
    if(kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())){
      slot0Configs.kS = kS.get();
      slot0Configs.kV = kV.get();
      slot0Configs.kP = kP.get();
      slot0Configs.kI = kI.get();
      slot0Configs.kD = kD.get();
      leftMotor.getConfigurator().apply(slot0Configs);
      rightMotor.getConfigurator().apply(slot0Configs);
    }
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
    return Math.abs(leftMotor.getClosedLoopError().getValueAsDouble()) <= rpmTolerance
      && Math.abs(rightMotor.getClosedLoopError().getValueAsDouble()) <= rpmTolerance;
  }

  /**
   * Set the speed of the flywheels. Remains at speed until called again.
   * @param rpm Flywheel RPM.
   * @param spinType Whether to use left or right motor to induce spin on note.
   */
  public void setSpeed(double rpm, SpinType spinType) {
    double rps = rpm / 60.0;
    leftMotor.setControl(velocityRequest.withVelocity(
      spinType == SpinType.slowLeftMotor ? rps * spinMultiplier.get() : rps
    ));
    rightMotor.setControl(velocityRequest.withVelocity(
      spinType == SpinType.slowRightMotor ? rps * spinMultiplier.get() : rps
    ));
  }
}
