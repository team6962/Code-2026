// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANdi;
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

/**
 * extends and retracts the thing that intakes the fuel
 */
public class IntakeExtension extends SubsystemBase {
  /**
   * checks wether the hall sensor has been triggered before, and lets us know if the machine knows the intake extension's postiion.
   */
  private boolean isZeroed = true;

  private CANdi candi;
  private TalonFX motor;

  private StatusSignal<Angle> distanceSignal;
  private StatusSignal<AngularVelocity> linearVelocitySignal;
  private StatusSignal<AngularAcceleration> linearAccelerationSignal;

  private StatusSignal<Voltage> voltageSignal;

  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Current> statorCurrentSignal;

  private StatusSignal<Boolean> candiTriggeredSignal;

  private StatusSignal<Double> closedLoopReferenceSignal;

  private IntakeExtensionSim simulation;

  public IntakeExtension() {
    candi = new CANdi(IntakeExtensionConstants.CANDI_DEVICE_ID, "subsystems");
    motor = new TalonFX(IntakeExtensionConstants.MOTOR_CAN_ID, new CANBus("subsystems"));
    motor.getConfigurator().apply(IntakeExtensionConstants.MOTOR_CONFIGURATION);
    candi.getConfigurator().apply(IntakeExtensionConstants.CANDI_CONFIGURATION);

    distanceSignal = motor.getPosition();
    linearVelocitySignal = motor.getVelocity();
    linearAccelerationSignal = motor.getAcceleration();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();
    candiTriggeredSignal = candi.getS1Closed();
    closedLoopReferenceSignal = motor.getClosedLoopReference();

    if (RobotBase.isSimulation()) {
      simulation = new IntakeExtensionSim(motor);
    }
  }

  /**
   * Makes sure the extension doesn't go over or under the maximum and minimum
   * @param input
   * @return Maximum, minimum, or the input
   */
  private Distance clampPositionToSafeRange(Distance input) {
    if (input.gt(IntakeExtensionConstants.MAX_POSITION)) {
      return IntakeExtensionConstants.MAX_POSITION;
    } else if (input.lt(IntakeExtensionConstants.MIN_POSITION)) {
      return IntakeExtensionConstants.MIN_POSITION;
    }
    return input;
  }

  public Command extend() {
    if (isZeroed) {
      return startEnd(
        () -> {
          motor.setControl(
            new MotionMagicVoltage(IntakeExtensionConstants.MAX_POSITION.in(Meters)));
        },
        () -> {
          motor.setControl(new MotionMagicVoltage(getPosition().in(Meters)));
        });
      }
      else {
        return null;
      }
  }

  public Command retract() {
    return startEnd(
        () -> {
          motor.setControl(
              new MotionMagicVoltage(IntakeExtensionConstants.MIN_POSITION.in(Meters)));
        },
        () -> {
          motor.setControl(new MotionMagicVoltage(getPosition().in(Meters)));
        });
  }

  /**
   * Finds the linear velocity of the end of the intake. Positive values mean that the intake is
   * extending outwards, and negative values mean that the intake is retracting inwards.
   *
   * @return The linear velocity as a LinearVelocity object.
   */
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(
        BaseStatusSignal.getLatencyCompensatedValue(
                linearVelocitySignal, linearAccelerationSignal)
            .in(RotationsPerSecond));
  }

  /**
   * Finds the position of the intake.
   *
   * @return The position as a Distance object.
   */
  public Distance getPosition() {
    return Meters.of(
        BaseStatusSignal.getLatencyCompensatedValue(distanceSignal, linearVelocitySignal)
            .in(Rotations));
  }

  /**
   * Finds the linear acceleration of the end of the intake.
   *
   * @return The linear acceleration as a LinearAcceleration object.
   */
  public LinearAcceleration getAcceleration() {
    return MetersPerSecondPerSecond.of(linearAccelerationSignal.getValueAsDouble());
  }

  /**
   * Voltage finds the voltage signal.
   *
   * @return It returns the voltage signal.
   */
  public Voltage getVoltage() {
    return voltageSignal.getValue();
  }

  /**
   * This method finds the stator current.
   *
   * @return It returns the current.
   */
  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  /**
   * This finds the supply current.
   *
   * @return It returns the supply current.
   */
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
  }

  /**
   * This finds out whether the CANdi was triggered
   *
   * @return true if hall sensor is triggered, false otherwise
   */
  private boolean getCANdiTriggered() {
    return candiTriggeredSignal.getValue();
  }

  private double getClosedLoopReference() {
    return closedLoopReferenceSignal.getValue();
  }

  @Override
  public void periodic() {
    if (simulation != null) {
      simulation.update();
    }
    else if (simulation == null) {
      motor.setPosition(IntakeExtensionConstants.MIN_POSITION.in(Meters));
    }
    if (getCANdiTriggered() && (getPosition().in(Meters) < IntakeExtensionConstants.MIN_POSITION.in(Meters))) {
      motor.setPosition(IntakeExtensionConstants.MIN_POSITION.in(Meters));
    }
     {
    if (getCANdiTriggered() != true) {
      isZeroed = false;
    }

    BaseStatusSignal.refreshAll(
        linearVelocitySignal,
        voltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        distanceSignal,
        linearAccelerationSignal,
        candiTriggeredSignal,
        closedLoopReferenceSignal);

    DogLog.log("Intake/Position", getPosition());
    DogLog.log("Intake/Velocity", getVelocity());
    DogLog.log("Intake/Acceleration", getAcceleration());
    DogLog.log("Intake/Voltage", getVoltage());
    DogLog.log("Intake/StatorCurrent", getStatorCurrent());
    DogLog.log("Intake/SupplyCurrent", getSupplyCurrent());
    DogLog.log("Intake/CandiTriggered", getCANdiTriggered());
    DogLog.log("Intake/ClosedLoopReference", getClosedLoopReference());
  }
  }
}
