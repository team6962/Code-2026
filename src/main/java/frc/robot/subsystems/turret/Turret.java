package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.math.AngleMath;
import com.team6962.lib.math.MeasureUtil;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/**
 * Subsystem for controlling the turret mechanism, which rotates the shooter around a vertical axis.
 * The turret uses a TalonFX motor controller and a hall effect sensor connected to a CANdi for
 * absolute position zeroing. The turret can be controlled using Motion Magic position control to
 * move to specific angles, and includes safety features to prevent movement before zeroing and to
 * clamp target angles within a safe range. The subsystem also includes extensive logging of status
 * signals and control requests for debugging and tuning purposes.
 */
public class Turret extends SubsystemBase {
  /** The motor that controls the turret */
  private TalonFX motor;

  /** The CANdi that connects the hall effect sensor to the CAN bus */
  private CANdi candi;

  // Status signals for logging and control purposes
  private StatusSignal<Angle> positionSignal;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<AngularAcceleration> accelerationSignal;
  private StatusSignal<Voltage> voltageSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Boolean> hallSensorTriggeredSignal;
  private StatusSignal<Double> profilePositionSignal;

  /** The simulation object that simulates the turret's behavior */
  private TurretSim simulation;

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

  /** The current motor configuration */
  private TalonFXConfiguration config;

  /** Assigns Status Signal variables to the different methods part of the motor.get() */
  public Turret() {
    // Assigns PID values and sets motor config
    motor = new TalonFX(TurretConstants.MOTOR_CAN_ID, new CANBus(TurretConstants.CAN_BUS_NAME));

    // Initialize hall effect sensor connected to the CANdi DIO port
    candi =
        new CANdi(
            TurretConstants.HALL_SENSOR_CANDI_CAN_ID, new CANBus(TurretConstants.CAN_BUS_NAME));

    // Configure the TalonFX motor controller
    config = new TalonFXConfiguration();

    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kD = TurretConstants.kD;
    config.Slot0.kS =
        RobotBase.isReal() ? TurretConstants.kS : 0; // No friction in simulation, so kS = 0
    config.Slot0.kV = RobotBase.isReal() ? TurretConstants.kV : TurretConstants.simulationKV;
    config.Slot0.kA = RobotBase.isReal() ? TurretConstants.kA : TurretConstants.simulationKA;

    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.MOTION_MAGIC_ACCELERATION;
    config.Feedback.SensorToMechanismRatio = TurretConstants.SENSOR_TO_MECHANISM_RATIO;

    config.MotorOutput.NeutralMode = TurretConstants.MOTOR_NEUTRAL_MODE;

    config.MotorOutput.Inverted = TurretConstants.MOTOR_INVERSION;

    config.CurrentLimits.StatorCurrentLimitEnable = RobotBase.isReal();
    config.CurrentLimits.StatorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT.in(Amps);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT.in(Amps);

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_ANGLE.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_ANGLE.in(Rotations);

    motor.getConfigurator().apply(config);

    // Initialize status signals
    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    accelerationSignal = motor.getAcceleration();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();
    hallSensorTriggeredSignal = candi.getS2Closed();
    profilePositionSignal = motor.getClosedLoopReference();

    // Tunable angle input, PID values, Motion Magic constraints
    DogLog.tunable(
        "Turret/Turret Position Target (Degrees)",
        getPosition().in(Degrees),
        newAngle -> {
          if (isZeroed()) {
            CommandScheduler.getInstance().schedule(moveTo(Degrees.of(newAngle)));
          }
        });

    DogLog.tunable(
        "Turret/Turret kP",
        config.Slot0.kP,
        newKP -> motor.getConfigurator().apply(config.Slot0.withKP(newKP)));

    DogLog.tunable(
        "Turret/Turret kD",
        config.Slot0.kD,
        newKD -> motor.getConfigurator().apply(config.Slot0.withKD(newKD)));

    DogLog.tunable(
        "Turret/Turret kS",
        config.Slot0.kS,
        newKS -> motor.getConfigurator().apply(config.Slot0.withKS(newKS)));

    DogLog.tunable(
        "Turret/Turret kV",
        config.Slot0.kV,
        newKV -> motor.getConfigurator().apply(config.Slot0.withKV(newKV)));

    DogLog.tunable(
        "Turret/Turret kA",
        config.Slot0.kA,
        newKA -> motor.getConfigurator().apply(config.Slot0.withKA(newKA)));

    DogLog.tunable(
        "Turret/Turret Cruise Velocity",
        config.MotionMagic.MotionMagicCruiseVelocity,
        newVel ->
            motor
                .getConfigurator()
                .apply(config.MotionMagic.withMotionMagicCruiseVelocity(newVel)));

    DogLog.tunable(
        "Turret/Turret Max Acceleration",
        config.MotionMagic.MotionMagicAcceleration,
        newAccel ->
            motor
                .getConfigurator()
                .apply(config.MotionMagic.withMotionMagicAcceleration(newAccel)));

    // Set motor to coast mode before it has been zeroed, to allow for easier manual zeroing
    motor.setControl(new CoastOut());

    if (RobotBase.isSimulation()) {
      simulation = new TurretSim(motor);

      // Assume the turret has been zeroed in simulation
      isZeroed = true;
    }
  }

  @Override
  public void periodic() {
    // Updates simulation each periodic cycle if a simulation object exists
    if (simulation != null) {
      simulation.update();
    }

    // If the robot is disabled, set the motor to hold its current position to prevent it from
    // moving
    if (RobotState.isDisabled() && isZeroed()) {
      setPositionControl(getPosition());
    }

    // Refresh all status signals
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        accelerationSignal,
        voltageSignal,
        statorCurrentSignal,
        supplyCurrentSignal,
        hallSensorTriggeredSignal,
        profilePositionSignal);

    // Check hall effect sensor and handle zeroing
    updateTurretAbsolutePosition();

    // Log all status signals and the current control request to NetworkTables
    DogLog.log("Turret/Position", getPosition());
    DogLog.log("Turret/Velocity", getVelocity());
    DogLog.log("Turret/Acceleration", getAcceleration());
    DogLog.log("Turret/AppliedVoltage", getAppliedVoltage());
    DogLog.log("Turret/StatorCurrent", getStatorCurrent());
    DogLog.log("Turret/SupplyCurrent", getSupplyCurrent());
    DogLog.log("Turret/HallSensorTriggered", isHallSensorTriggered());
    DogLog.log("Turret/IsZeroed", isZeroed());
    DogLog.log(
        "Turret/ProfilePosition",
        Rotations.of(profilePositionSignal.getValue()).in(Radians),
        Radians);

    LoggingUtil.log("Turret/ControlRequest", motor.getAppliedControl());
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
    if (RobotState.isEnabled() || isZeroed()) {
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
    }
  }

  /**
   * Gets the current position of the turret. An angle of 0 represents the turret facing the intake
   * side of the robot, and increasing angles rotate the turret counterclockwise when looking from
   * above.
   *
   * @return The current position of the turret as an Angle object
   */
  public Angle getPosition() {
    return MeasureUtil.toAngle(
        BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal));
  }

  /**
   * Gets the current angular velocity of the turret. Positive values indicate counterclockwise
   * rotation when looking from above.
   *
   * @return The current angular velocity of the turret as an AngularVelocity object
   */
  public AngularVelocity getVelocity() {
    return BaseStatusSignal.getLatencyCompensatedValue(velocitySignal, accelerationSignal);
  }

  /**
   * Gets the current angular acceleration of the turret.
   *
   * @return The current angular acceleration of the turret as an AngularAcceleration object
   */
  public AngularAcceleration getAcceleration() {
    return accelerationSignal.getValue();
  }

  /**
   * Gets the current applied voltage to the turret motor.
   *
   * @return The current applied voltage as a Voltage object
   */
  public Voltage getAppliedVoltage() {
    return voltageSignal.getValue();
  }

  /**
   * Gets the current stator current of the turret motor.
   *
   * @return The current stator current as a Current object
   */
  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  /**
   * Gets the current supply current of the turret motor.
   *
   * @return The current supply current as a Current object
   */
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
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
   * Clamps the given position to be within the safe range defined by TurretConstants.MIN_ANGLE and
   * TurretConstants.MAX_ANGLE. This is used to prevent commanding the turret to move beyond its
   * physical limits, which could cause damage to the mechanism or the motor.
   *
   * @param position The desired position to clamp
   * @return The clamped position within the safe range
   */
  private Angle clampPositionToSafeRange(Angle position) {
    if (position.lt(TurretConstants.MIN_ANGLE)) {
      return TurretConstants.MIN_ANGLE;
    } else if (position.gt(TurretConstants.MAX_ANGLE)) {
      return TurretConstants.MAX_ANGLE;
    } else {
      return position;
    }
  }

  /**
   * Optimizes a target angle to take the shortest path from the current angle, while also ensuring
   * the result is within the safe range defined by the given minimum and maximum angles.
   *
   * <p><b>IMPORTANT:</b> This method may output angles outside of the safe range if the turret's
   * safe range is less than 360 degrees.
   *
   * @param targetAngle The desired target angle to optimize
   * @param currentAngle The current angle of the turret, used to determine the shortest path
   * @param minAngle The minimum safe angle for the turret
   * @param maxAngle The maximum safe angle for the turret
   * @return The optimized target angle, which is usually within the safe range
   */
  public static Angle optimizeTarget(
      Angle targetAngle, Angle currentAngle, Angle minAngle, Angle maxAngle) {
    targetAngle = AngleMath.toDiscrete(targetAngle);
    targetAngle = AngleMath.toContinuous(targetAngle, currentAngle);

    if (targetAngle.gt(maxAngle)) {
      targetAngle = targetAngle.minus(Radians.of(2 * Math.PI));
    }

    if (targetAngle.lt(minAngle)) {
      targetAngle = targetAngle.plus(Radians.of(2 * Math.PI));
    }

    return targetAngle;
  }

  /**
   * Creates a Command that moves the turret to the specified target angle.
   *
   * @param targetAngle The target angle to move the turret to
   * @return A Command that moves the turret to the target angle when executed
   */
  public Command moveTo(Angle targetAngle) {
    return startEnd(
            () -> {
              Angle optimizedTargetPosition =
                  clampPositionToSafeRange(
                      optimizeTarget(
                          clampPositionToSafeRange(targetAngle),
                          getPosition(),
                          TurretConstants.MIN_ANGLE,
                          TurretConstants.MAX_ANGLE));
              setPositionControl(optimizedTargetPosition);
            },
            () -> {
              setPositionControl(getPosition());
            })
        .onlyIf(this::isZeroed);
  }

  /**
   * Creates a command that continuously drives the turret to a target angle provided by the given
   * supplier.
   *
   * @param targetAngleSupplier supplier that is sampled repeatedly to provide the desired target
   *     {@code Angle}
   * @return a {@code Command} that, while scheduled, continuously moves the turret toward the
   *     supplied target angle
   */
  public Command moveTo(Supplier<Angle> targetAngleSupplier) {
    return runEnd(
            () -> {
              Angle optimizedTargetPosition =
                  clampPositionToSafeRange(
                      optimizeTarget(
                          clampPositionToSafeRange(targetAngleSupplier.get()),
                          getPosition(),
                          TurretConstants.MIN_ANGLE,
                          TurretConstants.MAX_ANGLE));
              setPositionControl(optimizedTargetPosition);
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
    if (isZeroed()) {
      motor.setControl(new MotionMagicVoltage(targetAngle));
    } else if (RobotState.isEnabled()) {
      motor.setControl(new NeutralOut());
    } else {
      motor.setControl(new CoastOut());
    }
  }

  public Command moveAtVoltage(Voltage voltage) {
    return startEnd(
            () -> {
              if (isZeroed()) {
                motor.setControl(new VoltageOut(voltage));
              } else if (RobotState.isEnabled()) {
                motor.setControl(new NeutralOut());
              } else {
                motor.setControl(new CoastOut());
              }
            },
            () -> {
              setPositionControl(getPosition());
            })
        .onlyIf(() -> isZeroed());
  }
}
