package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
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
    motor = new TalonFX(ClimbConstants.MOTOR_ID); // motorID

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
      motor.setPosition(ClimbConstants.MAX_HEIGHT.in(Meters));
    }
  }

  // gets the acceleration signal in meters per second^2
  public LinearAcceleration getAcceleration() {
    return MetersPerSecondPerSecond.of(accelerationSignal.getValueAsDouble());
  }

  // gets the latency compensation neeeded with the velocity signal and acceleration signal
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(velocitySignal, accelerationSignal));
  }

  // gets the latency compensation needed with the current position signal and the velocity signal
  public Distance getPosition() {
    return Meters.of(
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(positionSignal, velocitySignal));
  }

  // gets the value of the voltage signal
  public Voltage getVoltage() {
    return voltageSignal.getValue();
  }

  // gets value of the stator current, which is the input current
  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  // gets value of the supply current, which is the total amount of current in the system
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
  }

  // determines if the hall sensor is triggered
  public boolean isHallEffectSensorTriggered() {
    return hallEffectSensorSignal.getValue();
  }

  @Override
  public void periodic() {
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

    if (isHallEffectSensorTriggered() && getPosition().lt(ClimbConstants.MIN_HEIGHT)) {
      motor.setPosition(ClimbConstants.MIN_HEIGHT.in(Meters));
      isZeroed = true;
    }

    if (simulation != null) {
      simulation.update();
    }
  }

  public Command elevate() {
    return startEnd(
        () -> {
          motor.setControl(new PositionVoltage(ClimbConstants.MAX_HEIGHT.in(Meters)));
        },
        () -> {
          motor.setControl(new PositionVoltage(getPosition().in(Meters)));
        });
  }

  public Command descend() {
    return startEnd(
        () -> {
          motor.setControl(new PositionVoltage(ClimbConstants.MIN_HEIGHT.in(Meters)));
        },
        () -> {
          motor.setControl(new PositionVoltage(getPosition().in(Meters)));
        });
  }

  public Command pullUp() {
    return startEnd(
        () -> {
          motor.setControl(new PositionVoltage(ClimbConstants.PULL_UP_HEIGHT.in(Meters)));
        },
        () -> {
          motor.setControl(new PositionVoltage(getPosition().in(Meters)));
        });
  }
}
