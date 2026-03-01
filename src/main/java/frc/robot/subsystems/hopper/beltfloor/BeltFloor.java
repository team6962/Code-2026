package frc.robot.subsystems.hopper.beltfloor;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

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

/**
 * The {@class BeltFloor} class is responsible for controlling the moving belt floor at the bottom
 * of the hopper that moves fuel into the queue.
 */
public class BeltFloor extends SubsystemBase {
  private TalonFX beltFloorMotor;

  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<AngularAcceleration> accelerationSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Voltage> voltageSignal;
  private BeltFloorSim simulation;

  /**
   * Initializes the motor controller, configures status signals for logging, and sets up DogLog
   * tunables for real-time testing.
   */
  public BeltFloor() {
    beltFloorMotor =
        new TalonFX(
            HopperConstants.BELT_FLOOR_MOTOR_CAN_ID, new CANBus(HopperConstants.CANBUS_NAME));
    beltFloorMotor.getConfigurator().apply(HopperConstants.BELT_FLOOR_MOTOR_CONFIG);

    /**
     * Assigning status signals to methods that retrieve the corresponding data from the motor
     * controller
     */
    velocitySignal = beltFloorMotor.getVelocity();
    voltageSignal = beltFloorMotor.getMotorVoltage();
    accelerationSignal = beltFloorMotor.getAcceleration();
    supplyCurrentSignal = beltFloorMotor.getSupplyCurrent();
    statorCurrentSignal = beltFloorMotor.getStatorCurrent();

    // Setup tunable dashboard control for testing
    DogLog.tunable(
        "Hopper/BeltFloor/AppliedVoltage",
        0.0,
        newVoltageDouble -> {
          Voltage target = edu.wpi.first.units.Units.Volts.of(newVoltageDouble);
          feedDump(target).schedule();
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
  private Command feedDump(Voltage targetVoltage) {

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

  /**
   * Moves the belts to feed fuel from the hopper into the queue.
   *
   * @return A command that runs the belt floor motor to feed fuel.
   */
  public Command feed() {
    return feedDump(Volts.of(8.0));
  }

  /**
   * Moves the belts in reverse.
   *
   * @return A command that runs the belt floor motor to move fuel away from the queue.
   */
  public Command reverse() {
    return feedDump(Volts.of(-6.0));
  }

  /**
   * Slowly runs the belts in reverse.
   *
   * @return A command that runs the belt floor motor at a low voltage to move fuel away from the
   *     queue.
   */
  public Command slowReverse() {
    return feedDump(Volts.of(-1.0));
  }

  /**
   * Moves the belts in reverse to dump fuel out of the hopper.
   *
   * @return A command that runs the belt floor motor to dump fuel.
   */
  public Command dump() {
    return feedDump(Volts.of(-8.0));
  }

  @Override
  public void periodic() {
    if (simulation != null) {
      simulation.update();
    }
    /**
     * this will log Voltage, AngularVelocity, BeltVelocity, StatorCurrent, AngularAcceleration, and
     * SupplyCurrent.
     */
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

  /**
   * gets linear velocity from multiplying the angular velocity by the circumference of the pulley
   */
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
