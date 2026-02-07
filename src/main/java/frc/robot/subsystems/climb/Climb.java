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

  public Climb() {
    motor = new TalonFX(30); // motorID

    motor.getConfigurator().apply(ClimbConstants.MOTOR_CONFIGURATION);

    candi = new CANdi(30, "subsystems"); // change later to correct CAN id
    candi.getConfigurator().apply(ClimbConstants.CANDI_CONFIGURATION);
    accelerationSignal = motor.getAcceleration();
    velocitySignal = motor.getVelocity();
    positionSignal = motor.getPosition();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();
    hallEffectSensorSignal = candi.getS1Closed(); // we don't know if it is sensor 1 or sensor 2
    if (RobotBase.isSimulation()) {
      simulation = new ClimbSim(motor);
    }
  }

  public LinearAcceleration getAcceleration() {
    return MetersPerSecondPerSecond.of(accelerationSignal.getValueAsDouble());
  }

  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(velocitySignal.getValueAsDouble());
  }

  public Distance getPosition() {
    return Meters.of(positionSignal.getValueAsDouble());
  }

  public Voltage getVoltage() {
    return voltageSignal.getValue();
  }

  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
  }

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
    DogLog.log("Climb/HallSignal", isHallEffectSensorTriggered());

    if (isHallEffectSensorTriggered()) {
      motor.setPosition(ClimbConstants.MIN_HEIGHT.in(Meters));
    }
  }

  public Command elevate() {
    return startEnd(
        () -> {
          motor.setControl(new PositionVoltage(8));
        },
        () -> {
          motor.setControl(new PositionVoltage(getPosition().in(Meters)));
        });
  }

  public Command descend() {
    return startEnd(
        () -> {
          motor.setControl(new PositionVoltage(0.0));
        },
        () -> {
          motor.setControl(new PositionVoltage(getPosition().in(Meters)));
        });
  }

  public Command pullUp() {
    return startEnd(
        () -> {
          motor.setControl(new PositionVoltage(0));
        },
        () -> {
          motor.setControl(new PositionVoltage(getPosition().in(Meters)));
        });
  }
}
