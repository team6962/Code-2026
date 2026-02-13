package frc.robot.subsystems.hopper.beltfloor;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants;

public class BeltFloor extends SubsystemBase {
  private TalonFX beltFloorMotor;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<AngularAcceleration> accelerationSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Voltage> voltageSignal;
  private BeltFloorSim simulation;

  public BeltFloor() {
    beltFloorMotor =
        new TalonFX(
            HopperConstants.BELT_FLOOR_MOTOR_CAN_ID,
            new CANBus(HopperConstants.BELT_FLOOR_CANBUS_NAME));
    beltFloorMotor.getConfigurator().apply(HopperConstants.BELT_FLOOR_MOTOR_CONFIG);
    velocitySignal = beltFloorMotor.getVelocity();
    voltageSignal = beltFloorMotor.getMotorVoltage();
    accelerationSignal = beltFloorMotor.getAcceleration();
    supplyCurrentSignal = beltFloorMotor.getSupplyCurrent();
    statorCurrentSignal = beltFloorMotor.getStatorCurrent();
    DogLog.tunable(
        "Hopper/BeltFloor/AppliedVoltage",
        0.0,
        newVoltage -> {
          feedDump(newVoltage).schedule();
        });
    if (RobotBase.isSimulation()) {
      simulation = new BeltFloorSim(beltFloorMotor);
    }
  }

  /**
   * this makes the motor go, positive voltage makes CAN ID 30 go counter clockwise, which is used
   * to feed fuel to the shooter. If you want to dump instead, use a negative voltage to make the
   * motor go clockwise.
   */
  public Command feedDump(double targetVoltage) { 

    return startEnd(
        () -> {
          // defines a local function to set motor voltage to make it go
          beltFloorMotor.setControl(new VoltageOut(targetVoltage));
        },
        () -> {
          // defines a local function to stop motor
          beltFloorMotor.setControl(new CoastOut());
        });
  }

  @Override
  public void periodic() {
    if (simulation != null) {
      simulation.update();
    }

    // this will log stuff every once in a while
    BaseStatusSignal.refreshAll(
        velocitySignal,
        voltageSignal,
        supplyCurrentSignal,
        statorCurrentSignal,
        accelerationSignal);
    DogLog.log("Hopper/BeltFloor/Voltage", getMotorVoltage());
    DogLog.log("Hopper/BeltFloor/AngularVelocity", getAngularVelocity());
    DogLog.log("Hopper/BeltFloor/BeltVelocity", getLinearVelocity());
    DogLog.log("Hopper/BeltFloor/StatorCurrent", getStatorCurrent());
    DogLog.log("Hopper/BeltFloor/AngularAcceleration", getAngularAcceleration());
    DogLog.log("Hopper/BeltFloor/SupplyCurrent", getSupplyCurrent());
  }

  /** gets the angular velocity */
  public AngularVelocity getAngularVelocity() {
    return velocitySignal.getValue();
  }

  /** gets the angular acceleration */
  public AngularAcceleration getAngularAcceleration() {
    return accelerationSignal.getValue();
  }

  /**gets linear velocity from multiplying the angular velocity by the circumference of the
     *pulley */
  public LinearVelocity getLinearVelocity() {
    return InchesPerSecond.of(
        (getAngularVelocity().in(RadiansPerSecond))
            * HopperConstants.BELT_FLOOR_PULLEY_RADIUS.in(Inches));
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
