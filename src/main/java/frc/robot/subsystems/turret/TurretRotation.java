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

/* This defines the Shooter Roller as a new Subsystem called motor.
 * The following StatusSignals represent certain stats of the motor */

public class TurretRotation extends SubsystemBase {
  private TalonFX motor;
  private StatusSignal<AngularVelocity> angVelocitySignal;
  private StatusSignal<Voltage> voltageSignal;
  private StatusSignal<Angle> angleSignal;
  private StatusSignal<AngularAcceleration> angAccelerationSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private TurretSim simulation;

  /* Assigns StatusSignals to different methods part of the motor.get...() */
  private final DoubleSubscriber angleInput;

  /** Assignes the Status Signal variables to the different methods part of the motor.get...() */
  public TurretRotation() {
    motor = new TalonFX(2, new CANBus("drivetrain"));
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = 0.5;
    config.Slot0.kD = 0.1;
    config.Slot0.kS = 0.150;
    config.Slot0.kV = 2.571;
    config.Slot0.kA = 0.030;

    config.MotionMagic.MotionMagicCruiseVelocity = 10;
    config.MotionMagic.MotionMagicAcceleration = 5;

    config.Feedback.RotorToSensorRatio = 150.0 / 7.0;

    motor.getConfigurator().apply(config);
    angVelocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    angleSignal = motor.getPosition();
    angAccelerationSignal = motor.getAcceleration();
    supplyCurrentSignal = motor.getSupplyCurrent();

    angleInput =
        DogLog.tunable(
            "Turret Rotation/Input Angle",
            0.0,
            newAngle -> {
              moveTo(Radians.of(newAngle)).schedule();
            });

    if (RobotBase.isSimulation()) {
      simulation = new TurretSim(motor);
    }
  }

  @Override
  public void periodic() {
    if (simulation != null) {
      simulation.update();
    }

    /* Motor Statistics are logged as Turret Rotation */
    BaseStatusSignal.refreshAll(
        angVelocitySignal, voltageSignal, angleSignal, angAccelerationSignal, supplyCurrentSignal);
    DogLog.log("Turret Rotation/Angular Velocity", getAngularVelocity());
    DogLog.log("Turret Rotation/Motor Voltage", getMotorVoltage());
    DogLog.log("Turret Rotation/Motor Position angle", getPosition());
    DogLog.log("Turret Rotation/Angular Acceleration", getAngularVelocity());
    DogLog.log("Turret Rotation/Angular Supply Current", getSupplyCurrent());

    LoggingUtil.log("Turret Rotation/Control Request", motor.getAppliedControl());
  }

  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
  }

  public AngularAcceleration getAcceleration() {
    return angAccelerationSignal.getValue();
  }

  public AngularVelocity getAngularVelocity() {
    return BaseStatusSignal.getLatencyCompensatedValue(angVelocitySignal, angAccelerationSignal);
  }

  public Voltage getMotorVoltage() {
    return voltageSignal.getValue();
  }

  public Angle getPosition() {
    return MeasureUtil.toAngle(
        BaseStatusSignal.getLatencyCompensatedValue(angleSignal, angVelocitySignal));
  }

  /* Moves the motor to the target angle */
  public Command moveToleft() {
    return startEnd(
        () -> motor.setControl(new MotionMagicVoltage(1)),
        () -> motor.setControl(new MotionMagicVoltage(0)));
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

  public Command moveTo(Angle targetAngle) {
    Angle clampedTargetPosition = clampPositionToSafeRange(targetAngle);
    return startEnd(
        () -> {
          motor.setControl(new MotionMagicVoltage(clampedTargetPosition));
        },
        () -> {
          motor.setControl(new MotionMagicVoltage(getPosition()));
        });
  }
}
