// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Based on 6328's TunableNumber and LoggedTunableNumber classes.
 * Updates while updateAll() is being called, then returns the last updated
 * value. If a TunableNumber already exists on NetworkTables (e.g. after a code
 * reset), will default to the existing value instead of user-provided default.
*/
public class TunableNumber implements AutoCloseable {
  private static ArrayList<TunableNumber> tunableNumbers = new ArrayList<TunableNumber>();
  private final String name;
  private double value;
  private Map<Integer, Double> lastCheckedValues = new HashMap<>();

  public TunableNumber(String name, double defaultValue) {
    this.name = name;
    value = defaultValue;
    /* Keep the current value if this exists on NetworkTables, otherwise
    * add it to NetworkTables with the default value. */
    SmartDashboard.putNumber(name, SmartDashboard.getNumber(name, defaultValue));
    tunableNumbers.add(this);
  }

  /** 
   * This is run automatically. Prevents 'updateAll()' from trying to update
   * a deleted TunableNumber.
   */
  @Override
  public void close() {
    tunableNumbers.remove(this);
  }

  /**
   * Update the value of all TunableNumbers.
   */
  public static void updateAll() {
    /* We want the numbers to update only while this function is called (e.g.
    * during disabledPeriodic()), so this function is used instead of checking
    * if the robot is disabled in hasChanged(), because hasChanged() may not be
    * called while the robot is disabled (e.g. in a default command's periodic
    * function), so a value changed during disabledPeriodic() would be ignored.
    */
    for (TunableNumber tunableNumber : tunableNumbers) {
      tunableNumber.update();
    }
  }

  /**
   * Checks whether the number has updated. Useful if the value being tuned is
   * expensive to change (e.g. creating a new PIDController), otherwise it is
   * safe to use get().
   * 
   * @param id Unique identifier for the caller to avoid conflicts when shared
   * between multiple objects. Recommended approach is to pass the result of
   * "hashCode()".
   * @return True if the number has updated since the last check, false
   * otherwise.
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastCheckedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastCheckedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  /**
   * Returns the current value of the TunableNumber. Will only update while
   * updateAll() is being called, otherwise returns the last updated value.
   * If updating the value is costly (e.g. creating a new PIDController), check
   * whether the value has changed using hasChanged().
   * @return Current value of TunableNumber
   */
  public double get() {
    return value;
  }

  protected void update() {
    value = SmartDashboard.getNumber(name, Double.NaN);
  }
}
