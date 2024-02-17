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
  Pose3d defaultShooterPose = new Pose3d(new Translation3d(-0.0208, 0, 0.1365), new Rotation3d());
  
  StructPublisher<Pose3d> shooterPublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Visualizer/2_Shooter_Pose", Pose3d.struct).publish();
  
  public void update() {
    double shooterAngleRad = RobotContainer.shooterAngle.getAngle().getRadians();
    shooterPublisher.set(defaultShooterPose.plus(new Transform3d(new Translation3d(), new Rotation3d(0, shooterAngleRad, 0))));
  }
}
