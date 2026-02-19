package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** climb class for the robot to climb the ladder at the during the competition */
public class Climb extends SubsystemBase {
  private TalonFX motor;
  private CANdi candi;
  private StatusSignal<AngularAcceleration> accelerationSignal;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<Angle> positionSignal;
  private StatusSignal<Voltage> voltageSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Boolean> hallEffectSensorSignal;
  private ClimbSim simulation;
  private boolean isZeroed;

  public Climb() {
    motor = new TalonFX(ClimbConstants.MOTOR_ID, ClimbConstants.CANBUS_NAME);

    motor.getConfigurator().apply(ClimbConstants.MOTOR_CONFIGURATION);

    candi = new CANdi(ClimbConstants.CANDI_CAN_ID, ClimbConstants.CANBUS_NAME);
    candi.getConfigurator().apply(ClimbConstants.CANDI_CONFIGURATION);

    accelerationSignal = motor.getAcceleration();
    velocitySignal = motor.getVelocity();
    positionSignal = motor.getPosition();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();
    hallEffectSensorSignal = candi.getS2Closed();
    if (RobotBase.isSimulation()) {
      simulation = new ClimbSim(motor);
      isZeroed = true;
    } else {
      motor.setPosition(ClimbConstants.MIN_HEIGHT.in(Meters));
    }
  }

  /**
   * Returns the current linear acceleration reported by the climb subsystem.
   *
   * <p>The value is read from the underlying acceleration signal and converted into a
   * LinearAcceleration expressed in meters per second squared (m/s²).
   *
   * @return the current linear acceleration in meters per second squared
   */
  public LinearAcceleration getAcceleration() {
    return MetersPerSecondPerSecond.of(accelerationSignal.getValueAsDouble());
  }

  /**
   * Returns the current climb velocity as a LinearVelocity measured in meters per second.
   *
   * <p>The returned value is produced by querying the underlying status signals and applying
   * latency compensation (using both the velocity and acceleration signals) before converting the
   * raw double result into a MetersPerSecond LinearVelocity instance.
   *
   * @return the latency-compensated climb velocity in meters per second
   * @throws NullPointerException if the underlying status signals are not initialized
   */
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(velocitySignal, accelerationSignal));
  }

  /**
   * Returns the current climb mechanism position as a Distance in meters.
   *
   * <p>The value is retrieved from the underlying BaseStatusSignal and is latency-compensated: the
   * raw position reading is adjusted using the associated velocity signal to estimate the position
   * at the current control-loop timestamp.
   *
   * <p>No state is modified by this call.
   *
   * @return the climb position as a Distance measured in meters (non-null)
   */
  public Distance getPosition() {
    return Meters.of(
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(positionSignal, velocitySignal));
  }

  /**
   * Returns the current voltage reading for the climb subsystem.
   *
   * <p>Retrieves the most recent Voltage value from the internal voltage signal. This represents
   * the instantaneous electrical potential reported by the subsystem's voltage source and can be
   * used for monitoring or control logic.
   *
   * @return the current Voltage reading from the climb subsystem
   */
  public Voltage getVoltage() {
    return voltageSignal.getValue();
  }

  /**
   * Returns the current measured at the motor stator for the climb subsystem.
   *
   * <p>This value is obtained from the underlying {@code statorCurrentSignal} provider and
   * represents the instantaneous stator current for the climb motor. Calling this method does not
   * modify the subsystem state.
   *
   * @return a {@code Current} object representing the measured stator current
   */
  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  /**
   * Returns the current supply current (the total current being drawn by the climb system).
   *
   * <p>The value is obtained from the internal {@code supplyCurrentSignal} and represents the
   * instantaneous electrical current supplied to the climb motor/system. This method does not
   * modify any subsystem state. Note that the returned {@link Current} reflects the most recently
   * sampled measurement; for up-to-date readings ensure the underlying signals are refreshed (for
   * example via {@code BaseStatusSignal.refreshAll(...)} in {@code periodic()}).
   *
   * @return a {@link Current} object representing the measured supply current (non-null)
   */
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
  }

  /**
   * Returns whether the Hall effect sensor is currently triggered.
   *
   * <p>This reads the underlying sensor signal and returns true when the sensor reports an
   * active/triggered state and false when it is not.
   *
   * @return true if the Hall effect sensor is triggered (active); false otherwise
   */
  public boolean isHallEffectSensorTriggered() {
    return hallEffectSensorSignal.getValue();
  }

  @Override
  public void periodic() {

    if (simulation != null) {
      simulation.update();
    }

    BaseStatusSignal.refreshAll(
        accelerationSignal,
        velocitySignal,
        positionSignal,
        voltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        hallEffectSensorSignal);

    DogLog.log("Climb/Acceleration", getAcceleration());
    DogLog.log("Climb/Velocity", getVelocity());
    DogLog.log("Climb/Position", getPosition());
    DogLog.log("Climb/Voltage", getVoltage());
    DogLog.log("Climb/StatorCurrent", getStatorCurrent());
    DogLog.log("Climb/SupplyCurrent", getSupplyCurrent());
    DogLog.log("Climb/HallSensorTriggered", isHallEffectSensorTriggered());

    if (RobotState.isDisabled()) {
      motor.setControl(new PositionVoltage(getPosition().in(Meters)));
    }

    if (isHallEffectSensorTriggered() && getPosition().lt(ClimbConstants.MIN_HEIGHT)) {
      motor.setPosition(ClimbConstants.MIN_HEIGHT.in(Meters));
      isZeroed = true;
    }
  }

  /**
   * Returns a command that raises the climb mechanism to its configured maximum height while
   * active.
   *
   * <p>On command start, the climb motor control is set to a position controller targeting {@code
   * ClimbConstants.MAX_HEIGHT} (converted to meters) using a PositionVoltage control mode. When the
   * command ends or is interrupted, the motor control is set to the current position (in meters) to
   * hold the mechanism in place and prevent drift.
   *
   * <p>The returned command follows a start-end lifecycle and does not automatically finish; it
   * should be scheduled and later canceled or interrupted by higher-level logic when the desired
   * behavior is complete.
   *
   * @return a {@code Command} that moves the climb to its maximum height while active and holds
   *     position when ended
   */
  public Command elevate() {
    return startEnd(
            () -> {
              motor.setControl(new PositionVoltage(ClimbConstants.MAX_HEIGHT.in(Meters)));
            },
            () -> {
              motor.setControl(new PositionVoltage(getPosition().in(Meters)));
            })
        .until(
            () ->
                getPosition().isNear(ClimbConstants.MAX_HEIGHT, ClimbConstants.POSITION_TOLERANCE))
        .onlyIf(() -> isZeroed);
  }

  /**
   * Creates a command that descends the climb mechanism to its configured minimum height.
   *
   * <p>On start, the command sets the climb motor control to a PositionVoltage target equal to
   * ClimbConstants.MIN_HEIGHT (converted to meters). When the command ends or is interrupted, it
   * sets the motor control to a PositionVoltage using the current measured position (in meters) so
   * the mechanism holds its current pose and does not continue moving.
   *
   * <p>This command is non-blocking and intended to be scheduled on the command scheduler.
   *
   * @return a Command which moves the climb to its minimum height and switches to position-hold on
   *     end
   */
  public Command descend() {
    return startEnd(
            () -> {
              motor.setControl(new PositionVoltage(ClimbConstants.MIN_HEIGHT.in(Meters)));
            },
            () -> {
              motor.setControl(new PositionVoltage(getPosition().in(Meters)));
            })
        .until(
            () ->
                getPosition().isNear(ClimbConstants.MIN_HEIGHT, ClimbConstants.POSITION_TOLERANCE));
  }

  /**
   * Creates a command that moves the climb mechanism to the configured "pull up" height.
   *
   * <p>When scheduled, the command sets the climb motor's control to a {@code PositionVoltage}
   * targeting {@code ClimbConstants.PULL_UP_HEIGHT} (converted to meters). When the command ends or
   * is interrupted, it resets the motor control to a {@code PositionVoltage} holding the climb's
   * current position.
   *
   * @return a {@code Command} which, while active, drives the climb to the pull-up height and, on
   *     end or interruption, commands the motor to hold its current position
   * @see ClimbConstants#PULL_UP_HEIGHT
   * @see PositionVoltage
   * @see #getPosition()
   */
  public Command pullUp() {
    return startEnd(
            () -> {
              motor.setControl(new PositionVoltage(ClimbConstants.PULL_UP_HEIGHT.in(Meters)));
            },
            () -> {
              motor.setControl(new PositionVoltage(getPosition().in(Meters)));
            })
        .onlyIf(() -> isZeroed);
  }

  /**
   * Creates a command that moves the climb mechanism at the given voltage. This command will do
   * nothing if the climb has not yet been zeroed and the voltage is positive.
   *
   * @param voltage The voltage to apply to the climb motor while the command is active. Positive
   *     voltages will cause the mechanism to extend, while negative voltages will cause it to
   *     retract.
   * @return a Command which applies the specified voltage to the climb motor while active and holds
   *     position on end
   */
  public Command moveAtVoltage(Voltage voltage) {
    return startEnd(
            () -> {
              motor.setControl(
                  new VoltageOut(
                      voltage.plus(Volts.of(ClimbConstants.MOTOR_CONFIGURATION.Slot0.kG))));
            },
            () -> {
              motor.setControl(new PositionVoltage(getPosition().in(Meters)));
            })
        .onlyIf(() -> isZeroed || voltage.in(Volts) < 0);
  }
}
