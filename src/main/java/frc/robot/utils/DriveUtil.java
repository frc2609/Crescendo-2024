// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Xbox;

/** Add your docs here. */
public class DriveUtil {
  /**
   * Some joysticks (like ours) map joystick positions to a square, meaning that if you use them to
   * drive the robot, the robot will try to move faster diagonally than normal.
   * This function maps the joystick values to a circle so the robot moves evenly.
   * @param x x value of joystick.
   * @param y y value of joystick.
   * @return Corrected joystick values. First element is x, second element is y.
   */
  public static Translation2d correctForSquareJoystickMapping(double xTranslation, double yTranslation) {
    // method from https://mathproofs.blogspot.com/2005/07/mapping-square-to-circle.html
    double xCorrected = xTranslation * Math.sqrt(1 - ((Math.pow(yTranslation, 2)) / 2));
    double yCorrected = yTranslation * Math.sqrt(1 - ((Math.pow(xTranslation, 2)) / 2));
    return new Translation2d(xCorrected, yCorrected);
  }

  /**
   * TODO: DOCS
   * @param controller Typically the driver controller.
   * @return Drivetrain sensitivity multiplier from 0 to 1.
   */
  public static double getSensitivity(CommandXboxController controller) {
    // define sensitivity calculations here, that way if they change, they will change everywhere
    boolean precision = controller.leftBumper().getAsBoolean() || controller.rightBumper().getAsBoolean();
    return precision ? 0.5 : 1.0;
  }

  /**
   * Get driver inputs from provided controller, performing some optional operations.
   * @param controller Typically the driver controller.
   * @param correctForSquareJoystickMapping True for our Xbox controllers.
   * @param cubeLinearXInput Set according to driver preference.
   * @param cubeLinearYInput Set according to driver preference. Leave this off for better left/right manuverability at high forward/back speeds.
   * @param cubeAngularVelocity Set according to driver preference.
   * @param sensitivityMultiplier Affects linear and angular velocities (not heading).
   * @return [linearX, linearY, angularVelocity, headingX, headingY]
   */
  public static double[] getDriverInputs(
    CommandXboxController controller,
    boolean correctForSquareJoystickMapping,
    boolean cubeLinearXInput,
    boolean cubeLinearYInput,
    boolean cubeAngularVelocity,
    double sensitivityMultiplier
  ) {
    // all values inverted because they are positive in the opposite direction
    // XboxController x and y are swapped from WPILib's field x and y
    double linearX = -MathUtil.applyDeadband(controller.getLeftY(), Xbox.joystickDeadband);
    double linearY = -MathUtil.applyDeadband(controller.getLeftX(), Xbox.joystickDeadband);
    // but they are not for the rotation joystick
    double headingX = -MathUtil.applyDeadband(controller.getRightX(), Xbox.joystickDeadband);
    double headingY = -MathUtil.applyDeadband(controller.getRightY(), Xbox.joystickDeadband);

    if (correctForSquareJoystickMapping) {
      Translation2d correctedSpeeds = correctForSquareJoystickMapping(linearX, linearY);
      linearX = correctedSpeeds.getX();
      linearY = correctedSpeeds.getY();
      Translation2d correctedHeadings = correctForSquareJoystickMapping(headingX, headingY);
      headingX = correctedHeadings.getX();
      headingY = correctedHeadings.getY();
    }

    double angularVelocity = headingX;
    
    if (cubeLinearXInput) linearX = Math.pow(linearX, 3);
    if (cubeLinearYInput) linearY = Math.pow(linearY, 3);
    if (cubeAngularVelocity) angularVelocity = Math.pow(angularVelocity, 3);

    linearX *= sensitivityMultiplier;
    linearY *= sensitivityMultiplier;
    angularVelocity *= sensitivityMultiplier;

    return new double[] {
      linearX,
      linearY,
      angularVelocity,
      headingX,
      headingY
    };
  }
}
