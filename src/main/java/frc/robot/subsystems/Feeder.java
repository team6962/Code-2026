// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Feeder controls the Feeder Subsystem on the robot.
 */
public class Feeder extends SubsystemBase {
  private TalonFX motorController;
  private StatusSignal<AngularVelocity> angularVelocitySignal;
  private StatusSignal<Voltage> voltageSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Current> statorCurrentSignal;

  public Feeder() {
    motorController = new TalonFX(16);
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.CurrentLimits.StatorCurrentLimitEnable = false;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
    motorController.getConfigurator().apply(configuration);
    angularVelocitySignal = motorController.getVelocity();
    voltageSignal = motorController.getMotorVoltage();
    statorCurrentSignal = motorController.getStatorCurrent();
    supplyCurrentSignal = motorController.getSupplyCurrent();
  }

  /**
   * move controls the voltage sent to the feeder. 
   * When the command starts, the voltage output will be four.
   * When the command stops, the code will cut off the voltage.
   * @return The command returned will rotate the feeder when on
   */
  public Command move() {
    return startEnd(() -> {
      motorController.setControl(new VoltageOut(5));
    }, () -> {
      motorController.setControl(new CoastOut());
    });
  }

  /**
   * AngularVelocity finds the angular velocity.
   * @return It returns the Angular Velocity.
   */
  public AngularVelocity getAngularVelocity() {
    return angularVelocitySignal.getValue();
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

  /**
   * This will now log all the information periodically into a dataset the will grow over time.
   */
  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(angularVelocitySignal, voltageSignal, statorCurrentSignal, supplyCurrentSignal);
    
    DogLog.log("feeder/angularVelocity", getAngularVelocity());
    DogLog.log("feeder/voltage", getVoltage());
    DogLog.log("feeder/statorCurrent", getStatorCurrent());
    DogLog.log("feeder/supplyCurrent", getSupplyCurrent());
    motorController.getSimState().setSupplyVoltage(12);
  }
}