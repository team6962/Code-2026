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
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.math.MeasureUtil;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Feeder controls the Feeder Subsystem on the robot.
 */
public class Intake extends SubsystemBase {
  private TalonFX motorController;
  private StatusSignal<AngularVelocity> angularVelocitySignal;
  private StatusSignal<Voltage> voltageSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<AngularAcceleration> angularAccelerationSignal;
  private StatusSignal<Angle> angleSignal;
  private IntakeSim simulation;

  public Intake() {
    motorController = new TalonFX(2, new CANBus("drivetrain"));
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.CurrentLimits.StatorCurrentLimitEnable = false;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
    configuration.Feedback.RotorToSensorRatio = 5.0 / 0.05;
    configuration.Slot0.kP = 0.33;
    motorController.getConfigurator().apply(configuration);
    angularVelocitySignal = motorController.getVelocity();
    voltageSignal = motorController.getMotorVoltage();
    statorCurrentSignal = motorController.getStatorCurrent();
    supplyCurrentSignal = motorController.getSupplyCurrent();
    angleSignal = motorController.getPosition();
    angularAccelerationSignal = motorController.getAcceleration();

    if (RobotBase.isSimulation()) {
        simulation = new IntakeSim(motorController);
    }
    
  }

  // public class ArmSim {
  //   private TalonFXSimState motorSim;
  //   private CANcoderSimSTate encoderSim;
  //   private SingleJointArmSim physicsSim;
  //   private double lastUpdateTimeSeconds = -1;
  //   private AngularVelocity lastVelocity = RadiansPerSecond.of(0);
  //   public ArmSim(TalonFX motor, CANcoder encoder) {

  public Command extend() {
    return startEnd(() -> {
      motorController.setControl(new PositionVoltage(1));
    }, () -> {
      motorController.setControl(new PositionVoltage(getDistance().in(Meters)));
    });
  }

  public Command retract() {
    return startEnd(() -> {
      motorController.setControl(new PositionVoltage(0));
    }, () -> {
      motorController.setControl(new PositionVoltage(getDistance(). magnitude()));
    });
  }

  public LinearVelocity getLinearVelocity() {
    //BaseStatusSignal.getLatencyCompensatedValue(angularVelocitySignal, angularAccelerationSignal);
    return MetersPerSecond.of(angularVelocitySignal.getValueAsDouble());
  }

  /**
   * Angle finds the position.
   * @return It returns the angle
   */
  public Distance getDistance() {
    return Meters.of(angleSignal.getValueAsDouble());
  }

  /**
   * Linear Acceleration finds the angular acceleration and converts it to linear acceleration
   * @return linear Acceleration
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

  /**
   * This will now log all the information periodically into a dataset the will grow over time.
   */
  @Override
  public void periodic() {
    if (simulation != null) {
      simulation.update();
    }
    BaseStatusSignal.refreshAll(angularVelocitySignal, voltageSignal, statorCurrentSignal, supplyCurrentSignal, angleSignal, angularAccelerationSignal);
    DogLog.log("intake/linearVelocity", getLinearVelocity());
    DogLog.log("intake/voltage", getVoltage());
    DogLog.log("intake/statorCurrent", getStatorCurrent());
    DogLog.log("intake/supplyCurrent", getSupplyCurrent());
    DogLog.log("intake/distance", getDistance());
    DogLog.log("intake/linearAcceleration", getAcceleration());
  }
}