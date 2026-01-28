// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX motor;

  private StatusSignal<Angle> angleSignal;
  private StatusSignal<AngularVelocity> angularVelocitySignal;
  private StatusSignal<AngularAcceleration> angularAccelerationSignal;

  private StatusSignal<Voltage> voltageSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Current> statorCurrentSignal;

  private IntakeSim simulation;

  public Intake() {
    motor = new TalonFX(2, new CANBus("drivetrain"));

    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.CurrentLimits.StatorCurrentLimitEnable = false;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
    configuration.Feedback.RotorToSensorRatio = 5.0 / 0.05;
    configuration.Slot0.kP = 0.36;
    motor.getConfigurator().apply(configuration);

    angleSignal = motor.getPosition();
    angularVelocitySignal = motor.getVelocity();
    angularAccelerationSignal = motor.getAcceleration();

    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();

    if (RobotBase.isSimulation()) {
        simulation = new IntakeSim(motor);
    }
  }

  public Command extend() {
    return startEnd(() -> {
      motor.setControl(new PositionVoltage(1));
    }, () -> {
      motor.setControl(new PositionVoltage(getPosition().in(Meters)));
    });
  }

  public Command retract() {
    return startEnd(() -> {
      motor.setControl(new PositionVoltage(0));
    }, () -> {
      motor.setControl(new PositionVoltage(getPosition().in(Meters)));
    });
  }

  /**
   * Finds the linear velocity of the end of the intake. Positive values mean
   * that the intake is extending outwards, and negative values mean that the intake
   * is retracting inwards.
   * 
   * @return The linear velocity as a LinearVelocity object.
   */
  public LinearVelocity getVelocity() {
    // TODO: Add latency compensation
    return MetersPerSecond.of(angularVelocitySignal.getValueAsDouble());
  }

  /**
   * Finds the position of the intake.
   * 
   * @return The position as a Distance object.
   */
  public Distance getPosition() {
    return Meters.of(angleSignal.getValueAsDouble());
  }

  /**
   * Finds the linear acceleration of the end of the intake.
   * 
   * @return The linear acceleration as a LinearAcceleration object.
   */
  public LinearAcceleration getAcceleration() {
    return MetersPerSecondPerSecond.of(angularAccelerationSignal.getValueAsDouble());
  }

  /**
   * Voltage finds the voltage signal.
   * @return It returns the voltage signal.
   */
  public Voltage getVoltage() {
    return voltageSignal.getValue();
  }

  /**
   * This method finds the stator current.
   * @return It returns the current.
   */
  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  /**
   * This finds the supply current.
   * @return It returns the supply current.
   */
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
  }

  @Override
  public void periodic() {
    if (simulation != null) {
      simulation.update();
    }

    BaseStatusSignal.refreshAll(angularVelocitySignal, voltageSignal, statorCurrentSignal, supplyCurrentSignal, angleSignal, angularAccelerationSignal);

    DogLog.log("intake/position", getPosition());
    DogLog.log("intake/velocity", getVelocity());
    DogLog.log("intake/acceleration", getAcceleration());

    DogLog.log("intake/voltage", getVoltage());
    DogLog.log("intake/statorCurrent", getStatorCurrent());
    DogLog.log("intake/supplyCurrent", getSupplyCurrent());
  }
}