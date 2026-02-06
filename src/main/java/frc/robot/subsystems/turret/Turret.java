package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.math.MeasureUtil;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This defines the Shooter Roller as a new Subsystem called motor. The following StatusSignals
 * represent certain stats of the motor
 */
public class Turret extends SubsystemBase {
  private TalonFX motor;
  private StatusSignal<AngularVelocity> angVelocitySignal;
  private StatusSignal<Voltage> voltageSignal;
  private StatusSignal<Angle> angleSignal;
  private StatusSignal<AngularAcceleration> angAccelerationSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private TurretSim simulation;

  /** The CANdi that connects the hall effect sensor to the CAN bus */
  private CANdi candi;

  /** The status signal that indicates whether the hall effect sensor is triggered */
  private StatusSignal<Boolean> hallSensorTriggeredSignal;

  /**
   * Indicates whether the turret has been zeroed using the hall effect sensor. When false, the
   * motor should not move and should be set to coast mode to allow for easier manual zeroing.
   */
  private boolean isZeroed = false;

  /**
   * Indicates whether the hall sensor has been triggered. Used to detect when the turret has
   * finished being zeroed.
   */
  private boolean hasHallSensorBeenTriggered = false;

  /* Assigns StatusSignals to different methods part of the motor.get...() */
  private final DoubleSubscriber angleInput;

  private final DoubleSubscriber kPInput;
  private final DoubleSubscriber kDInput;
  private final DoubleSubscriber kSInput;
  private final DoubleSubscriber kVInput;
  private final DoubleSubscriber kAInput;

  private final DoubleSubscriber cruiseVelocityInput;
  private final DoubleSubscriber accelerationInput;

  private TalonFXConfiguration config;

  /** Assigns Status Signal variables to the different methods part of the motor.get() */
  public Turret() {
    // Assigns PID values and sets motor config
    motor = new TalonFX(TurretConstants.MOTOR_CAN_ID, new CANBus(TurretConstants.CAN_BUS_NAME));

    // Initialize hall effect sensor connected to the CANdi DIO port
    candi = new CANdi(TurretConstants.HALL_SENSOR_CANDI, new CANBus(TurretConstants.CAN_BUS_NAME));
    hallSensorTriggeredSignal = candi.getS1Closed();

    config = new TalonFXConfiguration();

    // Assign PID and feedforward constants from TurretConstants
    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kD = TurretConstants.kD;
    config.Slot0.kS = TurretConstants.kS;
    config.Slot0.kV = TurretConstants.kV;
    config.Slot0.kA = TurretConstants.kA;

    // Assign motion magic constants
    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.MOTION_MAGIC_ACCELERATION;
    config.Feedback.SensorToMechanismRatio = TurretConstants.SENSOR_TO_MECHANISM_RATIO;

    // Apply neutral mode to use once the motor is zeroed. Before it is zeroed,
    // the motor will be set to coast mode to allow for easier manual zeroing.
    config.MotorOutput.NeutralMode = TurretConstants.MOTOR_NEUTRAL_MODE;

    // Apply motor inversion
    config.MotorOutput.Inverted = TurretConstants.MOTOR_INVERSION;

    // Apply current limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT.in(Amps);

    // Apply motor configs
    motor.getConfigurator().apply(config);

    // Initialize status signals
    angVelocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    angleSignal = motor.getPosition();
    angAccelerationSignal = motor.getAcceleration();
    supplyCurrentSignal = motor.getSupplyCurrent();

    // Tunable angle input, PID values, Motion Magic stuff
    angleInput =
        DogLog.tunable(
            TurretConstants.TUNABLE_ANGLE_KEY,
            TurretConstants.DEFAULT_ANGLE_INPUT,
            newAngle -> {
              if (isZeroed) {
                moveTo(Radians.of(newAngle)).schedule();
              }
            });

    kPInput =
        DogLog.tunable(
            TurretConstants.TUNABLE_KP_KEY,
            config.Slot0.kP,
            newKP -> {
              updatePIDConfig();
            });

    kDInput =
        DogLog.tunable(
            TurretConstants.TUNABLE_KD_KEY,
            config.Slot0.kD,
            newKD -> {
              updatePIDConfig();
            });

    kSInput =
        DogLog.tunable(
            TurretConstants.TUNABLE_KS_KEY,
            config.Slot0.kS,
            newKS -> {
              updatePIDConfig();
            });

    kVInput =
        DogLog.tunable(
            TurretConstants.TUNABLE_KV_KEY,
            config.Slot0.kV,
            newKV -> {
              updatePIDConfig();
            });

    kAInput =
        DogLog.tunable(
            TurretConstants.TUNABLE_KA_KEY,
            config.Slot0.kA,
            newKA -> {
              updatePIDConfig();
            });

    cruiseVelocityInput =
        DogLog.tunable(
            TurretConstants.TUNABLE_CRUISE_VELOCITY_KEY,
            config.MotionMagic.MotionMagicCruiseVelocity,
            newVel -> {
              updatePIDConfig();
            });

    accelerationInput =
        DogLog.tunable(
            TurretConstants.TUNABLE_ACCELERATION_KEY,
            config.MotionMagic.MotionMagicAcceleration,
            newAccel -> {
              updatePIDConfig();
            });

    // Set motor to coast mode before it has been zeroed, to allow for easier manual zeroing
    motor.setControl(new CoastOut());

    if (RobotBase.isSimulation()) {
      simulation = new TurretSim(motor);
      // Auto zero in sim
      isZeroed = true;
    }
  }

  /** Updates motor configuration with current tunable values */
  private void updatePIDConfig() {
    config.Slot0.kP = kPInput.get();
    config.Slot0.kD = kDInput.get();
    config.Slot0.kS = kSInput.get();
    config.Slot0.kV = kVInput.get();
    config.Slot0.kA = kAInput.get();

    config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocityInput.get();
    config.MotionMagic.MotionMagicAcceleration = accelerationInput.get();

    motor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // Updates simulation each periodic cycle if a simulation object exists
    if (simulation != null) {
      simulation.update();
    }

    // Refresh all status signals
    BaseStatusSignal.refreshAll(
        angVelocitySignal,
        voltageSignal,
        angleSignal,
        angAccelerationSignal,
        supplyCurrentSignal,
        hallSensorTriggeredSignal);

    // Check hall effect sensor and handle zeroing
    updateTurretAbsolutePosition();

    // Log all status signals and the current control request to NetworkTables
    DogLog.log(TurretConstants.LOG_ANGULAR_VELOCITY, getVelocity());
    DogLog.log(TurretConstants.LOG_MOTOR_VOLTAGE, getMotorVoltage());
    DogLog.log(TurretConstants.LOG_MOTOR_POSITION, getPosition());
    DogLog.log(TurretConstants.LOG_ANGULAR_ACCELERATION, getAcceleration());
    DogLog.log(TurretConstants.LOG_SUPPLY_CURRENT, getSupplyCurrent());
    DogLog.log(TurretConstants.LOG_IS_ZEROED, isZeroed);
    DogLog.log(TurretConstants.LOG_HALL_SENSOR_TRIGGERED, isHallSensorTriggered());

    LoggingUtil.log(TurretConstants.LOG_CONTROL_REQUEST, motor.getAppliedControl());
  }

  /**
   * Checks the hall effect sensor and updates the motor position and zeroing status accordingly.
   * This method should be called periodically to ensure the turret can zero itself using the hall
   * effect sensor.
   */
  private void updateTurretAbsolutePosition() {
    // Don't attempt to zero using the hall sensor if the robot is enabled or
    // if the turret has already been zeroed (which is always true in simulation),
    // to avoid interfering with normal operation
    if (RobotState.isEnabled() || isZeroed) {
      return;
    }

    // If the hall sensor is triggered but hasn't been triggered at all before, set the
    // motor position to the hall sensor angle
    if (isHallSensorTriggered() && !hasHallSensorBeenTriggered) {
      hasHallSensorBeenTriggered = true;

      // Determine which angle to use for initial zeroing
      Angle initialZeroAngle;
      if (TurretConstants.MINIMUM_HALL_SENSOR_TRIGGER_ANGLE == null
          && TurretConstants.MAXIMUM_HALL_SENSOR_TRIGGER_ANGLE == null) {
        throw new IllegalStateException(
            "At least one of MINIMUM_HALL_SENSOR_TRIGGER_ANGLE or MAXIMUM_HALL_SENSOR_TRIGGER_ANGLE must be non-null for hall sensor zeroing to work.");
      } else if (TurretConstants.MINIMUM_HALL_SENSOR_TRIGGER_ANGLE == null) {
        initialZeroAngle = TurretConstants.MAXIMUM_HALL_SENSOR_TRIGGER_ANGLE;
      } else if (TurretConstants.MAXIMUM_HALL_SENSOR_TRIGGER_ANGLE == null) {
        initialZeroAngle = TurretConstants.MINIMUM_HALL_SENSOR_TRIGGER_ANGLE;
      } else {
        initialZeroAngle =
            TurretConstants.MINIMUM_HALL_SENSOR_TRIGGER_ANGLE
                .plus(TurretConstants.MAXIMUM_HALL_SENSOR_TRIGGER_ANGLE)
                .div(2);
      }

      motor.setPosition(initialZeroAngle);
    }

    // If hall sensor is triggered at a lesser angle than ever before, set the
    // motor position to the minimum angle that can trigger the hall sensor
    if (isHallSensorTriggered()
        && TurretConstants.MINIMUM_HALL_SENSOR_TRIGGER_ANGLE != null
        && getPosition().lt(TurretConstants.MINIMUM_HALL_SENSOR_TRIGGER_ANGLE)) {
      motor.setPosition(TurretConstants.MINIMUM_HALL_SENSOR_TRIGGER_ANGLE);
    }

    // If hall sensor is triggered at a greater angle than ever before, set the
    // motor position to the maximum angle that can trigger the hall sensor
    if (isHallSensorTriggered()
        && TurretConstants.MAXIMUM_HALL_SENSOR_TRIGGER_ANGLE != null
        && getPosition().gt(TurretConstants.MAXIMUM_HALL_SENSOR_TRIGGER_ANGLE)) {
      motor.setPosition(TurretConstants.MAXIMUM_HALL_SENSOR_TRIGGER_ANGLE);
    }

    // If the hall sensor has exited the triggered range after being in it before,
    // the zeroing process is complete
    if (!isHallSensorTriggered() && hasHallSensorBeenTriggered) {
      isZeroed = true;

      // Set motor to normal neutral mode (brake mode) after zeroing is complete to signal
      // that the turret is ready for use and put it into its normal neutral state at the
      // beginning of the match
      motor.setControl(new NeutralOut());
    }
  }

  /**
   * Gets whether the turret has been zeroed using the hall effect sensor. When false, the turret
   * does not know its actual position and is not ready for match start.
   *
   * @return true if the turret is zeroed, false otherwise
   */
  public boolean isZeroed() {
    return isZeroed;
  }

  /**
   * Gets whether the hall sensor is currently triggered. This can be used to detect when the turret
   * is passing through the hall sensor range during zeroing.
   *
   * @return true if the hall sensor is triggered, false otherwise
   */
  public boolean isHallSensorTriggered() {
    return hallSensorTriggeredSignal.getValue();
  }

  /* Getter methods for logged values */
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
  }

  public AngularAcceleration getAcceleration() {
    return angAccelerationSignal.getValue();
  }

  public AngularVelocity getVelocity() {
    return BaseStatusSignal.getLatencyCompensatedValue(angVelocitySignal, angAccelerationSignal);
  }

  public Voltage getMotorVoltage() {
    return voltageSignal.getValue();
  }

  public Angle getPosition() {
    return MeasureUtil.toAngle(
        BaseStatusSignal.getLatencyCompensatedValue(angleSignal, angVelocitySignal));
  }

  private Angle clampPositionToSafeRange(Angle position) {
    if (position.lt(TurretConstants.MIN_ANGLE)) {
      return TurretConstants.MIN_ANGLE;
    } else if (position.gt(TurretConstants.MAX_ANGLE)) {
      return TurretConstants.MAX_ANGLE;
    } else {
      return position;
    }
  }

  /* Moves the motor based on an angle */
  public Command moveTo(Angle targetAngle) {
    Angle clampedTargetPosition = clampPositionToSafeRange(targetAngle);
    return startEnd(
            () -> {
              setPositionControl(clampedTargetPosition);
            },
            () -> {
              setPositionControl(getPosition());
            })
        .onlyIf(this::isZeroed);
  }

  /**
   * Applies a Motion Magic position control request to move the turret to the target angle. If the
   * turret is not yet zeroed, the motor is set to neutral mode instead.
   *
   * @param targetAngle The target angle to move the turret to
   */
  private void setPositionControl(Angle targetAngle) {
    if (isZeroed) {
      motor.setControl(new MotionMagicVoltage(targetAngle));
    } else {
      motor.setControl(new NeutralOut());
    }
  }
}
