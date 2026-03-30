package frc.robot.subsystems.hopper.floor;

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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperConstants;

public class RollerFloor extends SubsystemBase implements HopperFloor {
  private TalonFX rollerFloorMotor;

  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<AngularAcceleration> accelerationSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Voltage> voltageSignal;
  private RollerFloorSim simulation;

  public RollerFloor() {
    rollerFloorMotor =
        new TalonFX(
            HopperConstants.ROLLER_FLOOR_MOTOR_CAN_ID, new CANBus(HopperConstants.CANBUS_NAME));
    rollerFloorMotor.getConfigurator().apply(HopperConstants.ROLLER_FLOOR_MOTOR_CONFIG);

    /**
     * Assigning status signals to methods that retrieve the corresponding data from the motor
     * controller
     */
    velocitySignal = rollerFloorMotor.getVelocity();
    voltageSignal = rollerFloorMotor.getMotorVoltage();
    accelerationSignal = rollerFloorMotor.getAcceleration();
    supplyCurrentSignal = rollerFloorMotor.getSupplyCurrent();
    statorCurrentSignal = rollerFloorMotor.getStatorCurrent();

    // Setup tunable dashboard control for testing
    DogLog.tunable(
        "Hopper/RollerFloor/AppliedVoltage",
        0.0,
        newVoltageDouble -> {
          Voltage target = edu.wpi.first.units.Units.Volts.of(newVoltageDouble);
          CommandScheduler.getInstance().schedule(feedDump(target));
        });
    if (RobotBase.isSimulation()) {
      simulation = new RollerFloorSim(rollerFloorMotor);
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
          rollerFloorMotor.setControl(new VoltageOut(targetVoltage));
        },
        () -> {
          // defines a local function to stop motor
          rollerFloorMotor.setControl(new CoastOut());
        });
  }

  /**
   * Moves the rollers to feed fuel from the hopper into the queue.
   *
   * @return A command that runs the roller floor motor to feed fuel.
   */
  @Override
  public Command feed() {
    return feedDump(Volts.of(8.0));
  }

  /**
   * Moves the rollers in reverse.
   *
   * @return A command that runs the roller floor motor to move fuel away from the queue.
   */
  @Override
  public Command reverse() {
    return feedDump(Volts.of(-6.0));
  }

  /**
   * Slowly runs the rollers in reverse.
   *
   * @return A command that runs the roller floor motor at a low voltage to move fuel away from the
   *     queue.
   */
  @Override
  public Command slowReverse() {
    return feedDump(Volts.of(-1.0));
  }

  /**
   * Moves the rollers in reverse to dump fuel out of the hopper.
   *
   * @return A command that runs the roller floor motor to dump fuel.
   */
  @Override
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
    DogLog.log("Hopper/RollerFloor/Voltage", getMotorVoltage());
    DogLog.log("Hopper/RollerFloor/AngularVelocity", getAngularVelocity());
    DogLog.log("Hopper/RollerFloor/BeltVelocity", getLinearVelocity());
    DogLog.log("Hopper/RollerFloor/StatorCurrent", getStatorCurrent());
    DogLog.log("Hopper/RollerFloor/AngularAcceleration", getAngularAcceleration());
    DogLog.log("Hopper/RollerFloor/SupplyCurrent", getSupplyCurrent());
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
            * HopperConstants.ROLLER_FLOOR_PULLEY_RADIUS.in(Inches));
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
