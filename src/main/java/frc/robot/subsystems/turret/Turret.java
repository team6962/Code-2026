package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This defines the Shooter Roller as a new Subsystem called motor.
The following StatusSignals represent certain stats of the motor */

public class Turret extends SubsystemBase {
  private TalonFX motor;
  private StatusSignal<AngularVelocity> angVelocitySignal;
  private StatusSignal<Voltage> voltageSignal;
  private StatusSignal<Angle> angleSignal;
  private StatusSignal<AngularAcceleration> angAccelerationSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private TurretSim simulation;

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
    //Assigns PID values and sets motor config
    motor = new TalonFX(TurretConstants.MOTOR_CAN_ID, new CANBus(TurretConstants.CAN_BUS_NAME));
    config = new TalonFXConfiguration();
    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kD = TurretConstants.kD;
    config.Slot0.kS = TurretConstants.kS;
    config.Slot0.kV = TurretConstants.kV;
    config.Slot0.kA = TurretConstants.kA;

    // Assign motion magic constants
    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.MOTION_MAGIC_ACCELERATION;
    config.Feedback.SensorToMechanismRatio = TurretConstants.SENSOR_TO_MECHANISM_RATIO;

    // Apply motor configs and assign velocity signals
    motor.getConfigurator().apply(config);
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
              moveTo(Radians.of(newAngle)).schedule();
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

    if (RobotBase.isSimulation()) {
      simulation = new TurretSim(motor);
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
    // Updates simulation each periodic if a simulation exists
    if (simulation != null) {
      simulation.update();
    }

    /* Motor Statistics are refreshed logged as Turret Rotation each periodic */
    BaseStatusSignal.refreshAll(angVelocitySignal, voltageSignal, angleSignal, angAccelerationSignal, supplyCurrentSignal);
    DogLog.log(TurretConstants.LOG_ANGULAR_VELOCITY, getVelocity());
    DogLog.log(TurretConstants.LOG_MOTOR_VOLTAGE, getMotorVoltage());
    DogLog.log(TurretConstants.LOG_MOTOR_POSITION, getPosition());
    DogLog.log(TurretConstants.LOG_ANGULAR_ACCELERATION, getAcceleration());
    DogLog.log(TurretConstants.LOG_SUPPLY_CURRENT, getSupplyCurrent());

    LoggingUtil.log(TurretConstants.LOG_CONTROL_REQUEST, motor.getAppliedControl());
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

  /* Moves the motor to the left for testing */
  public Command moveToleft() {
    return startEnd(
        () -> motor.setControl(new MotionMagicVoltage(1)),
        () -> motor.setControl(new MotionMagicVoltage(0)));
  }

  /* Moves the motor based on an angle */
  public Command moveTo(Angle targetAngle) {
    return startEnd(
        () -> {
          motor.setControl(new MotionMagicVoltage(targetAngle));
        },
        () -> {
          motor.setControl(new MotionMagicVoltage(getPosition()));
        });
  }

  /* Moves the motor based on a double */
  public Command moveTo(double targetAngle) {
    return startEnd(
        () -> {
          motor.setControl(new MotionMagicVoltage(targetAngle));
        },
        () -> {
          motor.setControl(new MotionMagicVoltage(getPosition()));
        });
  }
}