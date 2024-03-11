package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterAngle2 extends SubsystemBase {
  private final CANSparkMax angleMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final SparkPIDController anglePID;

  public ShooterAngle2() {
    angleMotor.restoreFactoryDefaults();
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setInverted(false);
    angleMotor.setSmartCurrentLimit(40);
    anglePID = angleMotor.getPIDController();
    anglePID.setSmartMotionMaxVelocity(10000, 0);
    anglePID.setSmartMotionMinOutputVelocity(0, 0);
    anglePID.setSmartMotionMaxAccel(5000, 0);
    anglePID.setSmartMotionAllowedClosedLoopError(0.02, 0);
    anglePID.setP(0.000005);
    anglePID.setI(0.00001);
    anglePID.setD(0.00005);
    anglePID.setIZone(0.05);
    anglePID.setFF(0.00015);
    anglePID.setOutputRange(-1.0, 1.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle 1", angleMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Angle 2", angleMotor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {}


  public Command Raise() {
    return new InstantCommand(() -> anglePID.setReference(-8.0, CANSparkMax.ControlType.kSmartMotion));
  }

  public Command Lower() {
    return new InstantCommand(() -> anglePID.setReference(-14.0, CANSparkMax.ControlType.kSmartMotion));
  }

  public Command Stop() {
    return new InstantCommand(() -> angleMotor.set(0.0));
  }

}