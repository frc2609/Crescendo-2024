// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class LED extends SubsystemBase {
  // REV Blinkin pretends to be a PWM motor controller
  // private static final AddressableLED controller = new AddressableLED(2);
  private static Pattern pattern_drive;
  private static Pattern pattern_human;
  private static BlinkMode blinkMode_drive;
  private static BlinkMode blinkMode_human;
  private static AddressableLED led_dev;
  private static AddressableLEDBuffer led;
  private static int blinking_i = 0;
  private static int blinking_i_human = 0;

  public enum BlinkMode{
    SOLID,
    BLINKING_ON,
    BLINKING_OFF,
    FIRE,
    OFF;
  };
  public enum Pattern{
    INTAKE_NO_NOTE,
    INTAKE_NOTE,
    INTAKE_IDLE,
    RED,
    FIRE
  };

  public static final Map<Pattern, Color> PATTERN_MAP = new HashMap<Pattern, Color>(){{
      put(Pattern.INTAKE_NO_NOTE, new Color(50, 30, 0));
      put(Pattern.INTAKE_NOTE, new Color(0, 100, 0));
      put(Pattern.INTAKE_IDLE, new Color(50, 0, 50));
      put(Pattern.RED, new Color(50, 0, 0));
  }};
  int DRIVE_START = 0;
  int DRIVE_END = 46;
  final int HUMAN_START = 47;
  final int HUMAN_END = 91;
  
  public LED(){
    pattern_drive = Pattern.INTAKE_IDLE;
    pattern_human = Pattern.INTAKE_IDLE;
    blinkMode_drive = BlinkMode.SOLID;
    blinkMode_human = BlinkMode.SOLID;
    led_dev = new AddressableLED(1);
    led_dev.setLength(92);
    led = new AddressableLEDBuffer(92);
    setDrive(Pattern.INTAKE_IDLE, BlinkMode.SOLID);
    setHuman(Pattern.FIRE, BlinkMode.FIRE);
    // setHuman(Pattern.CUBE, BlinkMode.SOLID);
    setBuffer();
    led_dev.setData(led);
    led_dev.start();
  }
  public Color getMovingFireColor(int position, int totalLEDs) {
    // Calculate offset based on time for movement
    double time = Timer.getFPGATimestamp();
    int offset = (int)(time*30) % totalLEDs;

    // Adjust position by offset for movement
    position = (position + offset) % totalLEDs;

    // Your existing color gradient calculation
    float ratio = (float) position / totalLEDs;
    if (ratio < 0.5) {
      return interpolateColor(new Color(80, 80, 0), new Color(80, 20, 0), 0.5);
    } else {
      return interpolateColor(new Color(80, 20, 0), new Color(100, 0, 0), (ratio - 0.5) / 0.5);
    }
}
private Color interpolateColor(Color startColor, Color endColor, double ratio) {
  int red = (doubleToInt(startColor.red) + (int)(ratio * (doubleToInt(endColor.red) - doubleToInt(startColor.red))));
  int green = (doubleToInt(startColor.green) + (int)(ratio * (doubleToInt(endColor.green) - doubleToInt(startColor.green))));
  int blue = (doubleToInt(startColor.blue) + (int)(ratio * (doubleToInt(endColor.blue) - doubleToInt(startColor.blue))));
  return new Color(red, green, blue);
}

// Maps doubles from [0,1] to integers in [0,255]
private int doubleToInt(double zeroToOne){
  return (int)(zeroToOne*255.0);
}
  @Override
  public void periodic(){
    
    setBuffer();
    led_dev.setData(led);
  }

  public void setBuffer(){
    Color color = new Color(0, 0, 0);
    for(int i = DRIVE_START; i < DRIVE_END; i++){
      led.setLED(i, color);
    }
    switch(blinkMode_drive){
      case BLINKING_OFF:
        for(int i = DRIVE_START; i < DRIVE_END; i++){
          led.setLED(i, color);
        }
        if(blinking_i < 2){
          blinking_i++;
        }else{
          blinking_i = 0;
          blinkMode_drive = BlinkMode.BLINKING_ON;
        }
        break;
      case BLINKING_ON:
        color = PATTERN_MAP.getOrDefault(pattern_drive, new Color(0, 0, 0));
        for(int i = DRIVE_START; i < DRIVE_END; i++){
          led.setLED(i, color);
        }
        if(blinking_i < 2){
          blinking_i++;
        }else{
          blinking_i = 0;
          blinkMode_drive = BlinkMode.BLINKING_OFF;
        }
        break;
      case SOLID:
        color = PATTERN_MAP.getOrDefault(pattern_drive, new Color(0, 0, 0));
        for(int i = DRIVE_START; i < DRIVE_END; i++){
          led.setLED(i, color);
        }
        break;
      case OFF:
        color = new Color(0, 0, 0);
        for(int i = DRIVE_START; i < DRIVE_END; i++){
          led.setLED(i, color);
        }
        break;
      default:
        color = new Color(10, 10, 10);
        for(int i = DRIVE_START; i < DRIVE_END; i++){
          led.setLED(i, color);
        }
        DriverStation.reportError("INVALID LED STATE", null);
    }
    
    color = new Color(0, 0, 0);
    switch(blinkMode_human){
      case OFF:
        for(int i = HUMAN_START; i < HUMAN_END; i++){
          led.setLED(i, color);
        }
        break;
      case SOLID:
        color = PATTERN_MAP.getOrDefault(pattern_human, new Color(0, 0, 0));

        for(int i = HUMAN_START; i < HUMAN_END; i++){
          led.setLED(i, color);
        }
        break;
      case BLINKING_OFF:
      for(int i = HUMAN_START; i < HUMAN_END; i++){
        led.setLED(i, color);
      }
        if(blinking_i_human < 2){
          blinking_i_human++;
        }else{
          blinking_i_human = 0;
          blinkMode_human = BlinkMode.BLINKING_ON;
        }
        break;
      case BLINKING_ON:
        color = PATTERN_MAP.getOrDefault(pattern_human, new Color(0, 0, 0));
        for(int i = HUMAN_START; i < HUMAN_END; i++){
          led.setLED(i, color);
        }
        if(blinking_i_human < 2){
          blinking_i_human++;
        }else{
          blinking_i_human = 0;
          blinkMode_human = BlinkMode.BLINKING_OFF;
        }
        break;
      case FIRE:
        color = new Color(0, 0, 0);
        int length = HUMAN_END-HUMAN_START;
        for(int i = HUMAN_START; i < HUMAN_END; i++){
          color = getMovingFireColor(i-length,length);
          led.setLED(i, color);
        }
        break;
    }
  }

  /** Colour values are located in {@link frc.robot.Constants.LED Constants::LED}. */
  public void setDrive(Pattern pattern, BlinkMode blink) {
    pattern_drive = pattern;
    blinkMode_drive = blink;
  }

  public void setHuman(Pattern pattern, BlinkMode blink) {
    pattern_human = pattern;
    blinkMode_human = blink;
  }

  public void setIdle() {
    this.setDrive(Pattern.INTAKE_IDLE, BlinkMode.SOLID);
  }

  public void setNote() {
    this.setHuman(Pattern.INTAKE_NOTE, BlinkMode.SOLID);
  }

  public void setIntakeRunning() {
    this.setHuman(Pattern.INTAKE_NO_NOTE, BlinkMode.BLINKING_ON);
  }
}
