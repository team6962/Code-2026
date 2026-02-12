package frc.robot.subsystems.hopper.beltFloor;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class BeltFloor extends SubsystemBase{
    private TalonFX BeltFloorMotor;
    private StatusSignal<AngularVelocity> VelocitySignal;
    private StatusSignal<AngularAcceleration> AccelerationSignal;
    private StatusSignal<Current> supplyCurrentSignal;
    private StatusSignal<Current> statorCurrentSignal;
    private StatusSignal<Voltage> voltageSignal;
    private BeltFloorSim simulation;
    public BeltFloor() {
        BeltFloorMotor = 
            new TalonFX(HopperConstants.BELT_FLOOR_MOTOR_CAN_ID, new CANBus(HopperConstants.BELT_FLOOR_CANBUS_NAME));
        BeltFloorMotor.getConfigurator().apply(HopperConstants.BELT_FLOOR_MOTOR_CONFIGURATION);
        VelocitySignal = BeltFloorMotor.getVelocity();
        voltageSignal = BeltFloorMotor.getMotorVoltage();
        AccelerationSignal = BeltFloorMotor.getAcceleration();
        supplyCurrentSignal = BeltFloorMotor.getSupplyCurrent();
        statorCurrentSignal = BeltFloorMotor.getStatorCurrent();
        DogLog.tunable(
        "beltFloor / input voltage",
        0.0,
        newVoltage -> {
          roll(newVoltage).schedule();
        });
        if (RobotBase.isSimulation()) {
      simulation = new BeltFloorSim(BeltFloorMotor);
    }
    }

    public Command roll(double targetVoltage) { //name temporary change later

    return startEnd(
        () -> {
          // defines a local function to set motor voltage to make it go
          BeltFloorMotor.setControl(new VoltageOut(targetVoltage));
        },
        () -> {
          // defines a local function to stop motor
          BeltFloorMotor.setControl(new CoastOut());
        });
  }

  @Override
  public void periodic() {
    if (simulation != null) {
      simulation.update();
    }

    // this will log stuff every once in a while
    BaseStatusSignal.refreshAll(
        VelocitySignal,
        voltageSignal,
        supplyCurrentSignal,
        statorCurrentSignal,
        AccelerationSignal);
    DogLog.log("beltFloor/voltage", getMotorVoltage());
    DogLog.log("beltFloor/angularVelocity", getAngularVelocity());
    DogLog.log("beltFloor/statorCurrent", getStatorCurrent());
    DogLog.log("beltFloor/angularAcceleration", getAngularAcceleration());
    DogLog.log("beltFloor/supplyCurrent", getSupplyCurrent());
  }

  /** gets the angular velocity */
  public AngularVelocity getAngularVelocity() {
    return VelocitySignal.getValue();
  }

  /** gets the angular acceleration */
  public AngularAcceleration getAngularAcceleration() {
    return AccelerationSignal.getValue();
  }

  /** gets the supply current */
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
  }

  /** gets the stator current */
  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  /** gets the motor voltage */
  public Voltage getMotorVoltage() {
    return voltageSignal.getValue();
  }
}


