// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private List<Segment> segments;
  private static AddressableLED led_dev;
  private static AddressableLEDBuffer led;

  private class Segment {
    String name;
    int start;
    int end;
    Pattern pattern;
    BlinkMode blinkMode;
    int blinking_i;

    public Segment(String name, int start, int end, Pattern pattern, BlinkMode blinkMode) {
      this.name = name;
      this.start = start;
      this.end = end;
      this.pattern = pattern;
      this.blinkMode = blinkMode;
      this.blinking_i = 0;
    }
  }

  public enum BlinkMode {
    SOLID,
    BLINKING_ON,
    BLINKING_OFF,
    FIRE,
    OFF;
  };

  public enum Pattern {
    INTAKE_NO_NOTE,
    INTAKE_NOTE,
    INTAKE_IDLE,
    RED,
    FIRE,
    WHITE
  };

  public static final Map<Pattern, Color> PATTERN_MAP = new HashMap<Pattern, Color>() {{
    put(Pattern.INTAKE_NO_NOTE, new Color(50, 30, 0));
    put(Pattern.INTAKE_NOTE, new Color(0, 80, 0));
    put(Pattern.INTAKE_IDLE, new Color(50, 0, 50));
    put(Pattern.RED, new Color(50, 0, 0));
  }};
  
  public LED() {
    segments = new ArrayList<>();
    segments.add(new Segment("drive", 0, 27, Pattern.INTAKE_IDLE, BlinkMode.SOLID));
    segments.add(new Segment("align", 28, 34, Pattern.INTAKE_IDLE, BlinkMode.SOLID));
    segments.add(new Segment("flywheel", 35, 40, Pattern.INTAKE_IDLE, BlinkMode.SOLID));
    segments.add(new Segment("angle", 41, 46, Pattern.INTAKE_IDLE, BlinkMode.SOLID));
    segments.add(new Segment("human", 47, 91, Pattern.INTAKE_IDLE, BlinkMode.SOLID));
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

  @Override
  public void periodic() {
    setBuffer();
    led_dev.setData(led);
  }

  public Color getMovingFireColor(int position, int totalLEDs) {
    // Calculate offset based on time for movement
    double time = Timer.getFPGATimestamp();
    int offset = (int) (time * 30) % totalLEDs;

    // Adjust position by offset for movement
    position = (position + offset) % totalLEDs;

    // Calculate color gradient
    float ratio = (float) position / totalLEDs;
    if (ratio < 0.5) {
      return interpolateColor(new Color(80, 80, 0), new Color(80, 20, 0), 0.5);
    } else {
      return interpolateColor(new Color(80, 20, 0), new Color(100, 0, 0), (ratio - 0.5) / 0.5);
    }
  }

  private Color interpolateColor(Color startColor, Color endColor, double ratio) {
    int red = (doubleToInt(startColor.red) + (int) (ratio * (doubleToInt(endColor.red) - doubleToInt(startColor.red))));
    int green = (doubleToInt(startColor.green) + (int) (ratio * (doubleToInt(endColor.green) - doubleToInt(startColor.green))));
    int blue = (doubleToInt(startColor.blue) + (int) (ratio * (doubleToInt(endColor.blue) - doubleToInt(startColor.blue))));
    return new Color(red, green, blue);
  }

  // Maps doubles from [0,1] to integers in [0,255]
  private int doubleToInt(double zeroToOne) {
    return (int) (zeroToOne * 255.0);
  }

  public void setBuffer() {
    for (Segment segment : segments) {
      Color color = new Color(0, 0, 0);
      switch (segment.blinkMode) {
        case BLINKING_OFF:
          color = new Color(0, 0, 0);
          for (int i = segment.start; i < segment.end; i++) {
            led.setLED(i, color);
          }
          if (segment.blinking_i < 2) {
            segment.blinking_i++;
          } else {
            segment.blinking_i = 0;
            segment.blinkMode = BlinkMode.BLINKING_ON;
          }
          break;
        case BLINKING_ON:
          color = PATTERN_MAP.getOrDefault(segment.pattern, new Color(0, 0, 0));
          for (int i = segment.start; i < segment.end; i++) {
            led.setLED(i, color);
          }
          if (segment.blinking_i < 2) {
            segment.blinking_i++;
          } else {
            segment.blinking_i = 0;
            segment.blinkMode = BlinkMode.BLINKING_OFF;
          }
          break;
        case SOLID:
          color = PATTERN_MAP.getOrDefault(segment.pattern, new Color(0, 0, 0));
          for (int i = segment.start; i < segment.end; i++) {
            led.setLED(i, color);
          }
          break;
        case OFF:
          color = new Color(0, 0, 0);
          for (int i = segment.start; i < segment.end; i++) {
            led.setLED(i, color);
          }
          break;
        case FIRE:
          color = new Color(0, 0, 0);
          int length = segment.end - segment.start;
          for (int i = segment.start; i < segment.end; i++) {
            color = getMovingFireColor(i - segment.start, length);
            led.setLED(i, color);
          }
          break;
        default:
          color = new Color(10, 10, 10);
          for (int i = segment.start; i < segment.end; i++) {
            led.setLED(i, color);
          }
          DriverStation.reportError("INVALID LED STATE", null);
      }
    }
  }

  public void addSegment(Segment segment) {
    segments.add(segment);
  }

  public void setSegmentPattern(String segmentName, Pattern pattern, BlinkMode blinkMode) {
    for (Segment segment : segments) {
      if (segment.name.equals(segmentName)) {
        segment.pattern = pattern;
        segment.blinkMode = blinkMode;
        break;
      }
    }
  }

  public void setDrive(Pattern pattern, BlinkMode blink) {
    setSegmentPattern("drive", pattern, blink);
  }

  public void setHuman(Pattern pattern, BlinkMode blink) {
    setSegmentPattern("human", pattern, blink);
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
