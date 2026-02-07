package frc.robot.subsystems.shooterrollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** this is the subsystem for the flywheels that both makes the motor go and records motor values */
public class ShooterRollers extends SubsystemBase {
  private TalonFX shooterRollerMotor1;
  private TalonFX shooterRollerMotor2;
  private StatusSignal<AngularVelocity> VelocitySignal;
  private StatusSignal<AngularAcceleration> AccelerationSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Voltage> voltageSignal;
  private ShooterRollersSim simulation;

  public ShooterRollers() {
    shooterRollerMotor1 =
        new TalonFX(
            ShooterRollersConstants.MOTOR_CAN_ID_1, new CANBus(ShooterRollersConstants.CANBUS_NAME));

    shooterRollerMotor1.getConfigurator().apply(ShooterRollersConstants.MOTOR_CONFIGURATION);
    shooterRollerMotor2 =
        new TalonFX(
            ShooterRollersConstants.MOTOR_CAN_ID_2, new CANBus(ShooterRollersConstants.CANBUS_NAME));

    shooterRollerMotor2.getConfigurator().apply(ShooterRollersConstants.MOTOR_CONFIGURATION);
    // defines the variables we are keeping track of
    VelocitySignal = shooterRollerMotor1.getVelocity();
    voltageSignal = shooterRollerMotor1.getMotorVoltage();
    AccelerationSignal = shooterRollerMotor1.getAcceleration();
    supplyCurrentSignal = shooterRollerMotor1.getSupplyCurrent();
    statorCurrentSignal = shooterRollerMotor1.getStatorCurrent();

    DogLog.tunable(
        "shooterRoller / input velocity",
        0.0,
        newVelocity -> {
          CommandScheduler.getInstance().schedule(shoot(newVelocity));
        });

    shooterRollerMotor2.setControl(
        new Follower(shooterRollerMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    if (RobotBase.isSimulation()) {
      simulation = new ShooterRollersSim(shooterRollerMotor1);
    }
  }

  /**
   * this makes the motor go, positive voltage makes CAN ID 21 go counter clockwise, which is not
   * what we want, please put a negative value so it goes clockwise since thats what is intended by
   * the build team
   */
  public Command shoot(double targetVelocity) {

    return startEnd(
        () -> {
          // defines a local function to set motor voltage to make it go
          shooterRollerMotor1.setControl(new VelocityVoltage(targetVelocity));
        },
        () -> {
          // defines a local function to stop motor
          shooterRollerMotor1.setControl(new CoastOut());
        });
  }

  @Override
  public void periodic() {
    if (simulation != null) {
      simulation.update();
    }

    // this will log stuff every once in a while
    BaseStatusSignal.refreshAll(
        VelocitySignal,
        voltageSignal,
        supplyCurrentSignal,
        statorCurrentSignal,
        AccelerationSignal);
    DogLog.log("shooterRoller/voltage", getMotorVoltage());
    DogLog.log("shooterRoller/angularVelocity", getAngularVelocity());
    DogLog.log("shooterRoller/statorCurrent", getStatorCurrent());
    DogLog.log("shooterRoller/angularAcceleration", getAngularAcceleration());
    DogLog.log("shooterRoller/supplyCurrent", getSupplyCurrent());
  }

  /** gets the angular velocity */
  public AngularVelocity getAngularVelocity() {
    return VelocitySignal.getValue();
  }

  /** gets the angular acceleration */
  public AngularAcceleration getAngularAcceleration() {
    return AccelerationSignal.getValue();
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
