// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class Visualizer {
  final Pose3d defaultIntakePose = new Pose3d(new Translation3d(0.130, 0, 0.076), new Rotation3d());
  final Pose3d defaultElevatorPose = new Pose3d(new Translation3d(0.130, 0, 0.026), new Rotation3d());
  final Pose3d defaultShooterPose = new Pose3d(new Translation3d(-0.0208, 0, 0.1365), new Rotation3d());
  final Pose3d defaultLaserPose = new Pose3d(new Translation3d(0, 0, 0.16), new Rotation3d()); 

  StructPublisher<Pose3d> intakePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Visualizer/0_Intake_Pose", Pose3d.struct).publish();

  StructPublisher<Pose3d> elevatorPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Visualizer/1_Elevator_Pose", Pose3d.struct).publish();

  StructPublisher<Pose3d> shooterPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Visualizer/2_Shooter_Pose", Pose3d.struct).publish();
  
  StructPublisher<Pose3d> laserPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Visualizer/3_Targeting_Laser_Pose", Pose3d.struct).publish();
  
  public void update() {
    double liftMeters = 0;
    double maxIntakeHeightPlaceholder = 0.4;
    
    intakePublisher.set(defaultIntakePose.plus(new Transform3d(new Translation3d(0.0, 0.0, liftMeters), new Rotation3d())));
    elevatorPublisher.set(defaultElevatorPose.plus(new Transform3d(new Translation3d(0.0, 0.0, Math.max(liftMeters - maxIntakeHeightPlaceholder, 0)), new Rotation3d())));
    double shooterAngleRad = RobotContainer.shooterAngle.getAngle().getRadians();
    shooterPublisher.set(defaultShooterPose.plus(new Transform3d(new Translation3d(), new Rotation3d(0, shooterAngleRad, 0))));
    laserPublisher.set(defaultLaserPose.plus(new Transform3d(new Translation3d(), new Rotation3d(0, shooterAngleRad, 0))));
  }
}
