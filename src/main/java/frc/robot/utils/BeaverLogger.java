// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.function.Supplier;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeaverLogger {
  HashMap<String, Supplier<Double>> loggingMap = new HashMap<String, Supplier<Double>>();
  HashMap<String, Supplier<Double>> smartDashboardMap = new HashMap<String, Supplier<Double>>();
  HashMap<String, DoubleLogEntry> logEntryMap = new HashMap<String, DoubleLogEntry>();
  final DataLog log = DataLogManager.getLog();

  public BeaverLogger() {}

  public void addLoggable(String name, Supplier<Double> supplier, boolean toSmartDash) {
    if (toSmartDash) {
      smartDashboardMap.put(name, supplier);
    } else {
      loggingMap.put(name, supplier);
      logEntryMap.put(name, new DoubleLogEntry(log, name));
    }
  }

  public void logAll() {
    // Logging values in smartDashboardMap
    for (String key : smartDashboardMap.keySet()) {
        Double value = smartDashboardMap.get(key).get();
        SmartDashboard.putNumber(key, value);
        // You can use your actual method for sending data to SmartDashboard here
    }
    // Logging values in loggingMap
    for (String key : loggingMap.keySet()) {
        Double value = loggingMap.get(key).get();
        DoubleLogEntry entry = logEntryMap.get(key);
      if (entry == null) {
        DataLogManager.log(String.format("key: %s not found in logEntryMap", key));
      } else {
        entry.append(value);
      }
    }
  }
}
