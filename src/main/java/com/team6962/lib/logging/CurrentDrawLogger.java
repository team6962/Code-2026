package com.team6962.lib.logging;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class CurrentDrawLogger extends SubsystemBase {
  private static List<Device> devices = new ArrayList<>();

  public static void start() {
    new CurrentDrawLogger();
  }

  public static void add(String name, Supplier<Current> currentSupplier) {
    synchronized (devices) {
      devices.add(new Device(name, currentSupplier));
    }
  }

  public static Current getTotalCurrent() {
    Current totalCurrent = Amps.of(0);

    synchronized (devices) {
      for (Device device : devices) {
        totalCurrent = totalCurrent.plus(device.getCurrent());
      }
    }

    return totalCurrent;
  }

  @Override
  public void periodic() {
    Voltage batteryVoltage = Volts.of(RobotController.getBatteryVoltage());
    Current totalCurrent = getTotalCurrent();

    DogLog.log("CurrentDraw/BatteryVoltage", batteryVoltage);
    DogLog.log("CurrentDraw/TotalCurrent", totalCurrent);

    for (Device device : devices) {
      DogLog.log("CurrentDraw/" + device.getName(), device.getCurrent());
    }
  }

  private static class Device {
    private final String name;
    private final Supplier<Current> currentSupplier;

    public Device(String name, Supplier<Current> currentSupplier) {
      this.name = name;
      this.currentSupplier = currentSupplier;
    }

    public String getName() {
      return name;
    }

    public Current getCurrent() {
      return currentSupplier.get();
    }
  }
}
