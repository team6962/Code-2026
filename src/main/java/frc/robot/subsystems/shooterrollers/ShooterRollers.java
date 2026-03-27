package frc.robot.subsystems.shooterrollers;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team6962.lib.logging.CurrentDrawLogger;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.Supplier;

/** this is the subsystem for the flywheels that both makes the motor go and records motor values */
public class ShooterRollers extends SubsystemBase {
  private TalonFX shooterRollerMotor1;
  private TalonFX shooterRollerMotor2;
  private StatusSignal<Angle> angleSignal;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<AngularAcceleration> accelerationSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Current> otherSupplyCurrentSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Voltage> voltageSignal;
  private ShooterRollersSim simulation;

  public ShooterRollers() {
    shooterRollerMotor1 =
        new TalonFX(
            ShooterRollersConstants.MOTOR_CAN_ID_1,
            new CANBus(ShooterRollersConstants.CANBUS_NAME));

    shooterRollerMotor1.getConfigurator().apply(ShooterRollersConstants.MOTOR_CONFIGURATION);

    shooterRollerMotor2 =
        new TalonFX(
            ShooterRollersConstants.MOTOR_CAN_ID_2,
            new CANBus(ShooterRollersConstants.CANBUS_NAME));

    // Second motor is inverted from the first motor
    ShooterRollersConstants.MOTOR_CONFIGURATION.MotorOutput.Inverted =
        ShooterRollersConstants.MOTOR_CONFIGURATION.MotorOutput.Inverted
                == InvertedValue.Clockwise_Positive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    shooterRollerMotor2.getConfigurator().apply(ShooterRollersConstants.MOTOR_CONFIGURATION);

    // defines the variables we are keeping track of
    angleSignal = shooterRollerMotor1.getPosition();
    velocitySignal = shooterRollerMotor1.getVelocity();
    voltageSignal = shooterRollerMotor1.getMotorVoltage();
    accelerationSignal = shooterRollerMotor1.getAcceleration();
    supplyCurrentSignal = shooterRollerMotor1.getSupplyCurrent();
    statorCurrentSignal = shooterRollerMotor1.getStatorCurrent();
    otherSupplyCurrentSignal = shooterRollerMotor2.getSupplyCurrent();

    DogLog.tunable(
        "shooterRoller / input velocity",
        0.0,
        newVelocity -> {
          CommandScheduler.getInstance().schedule(shoot(RotationsPerSecond.of(newVelocity)));
        });

    DogLog.tunable(
        "shooterRoller/kS",
        ShooterRollersConstants.MOTOR_CONFIGURATION.Slot0.kS,
        value -> {
          shooterRollerMotor1
              .getConfigurator()
              .apply(ShooterRollersConstants.MOTOR_CONFIGURATION.Slot0.withKS(value));
        });

    DogLog.tunable(
        "shooterRoller/kV",
        ShooterRollersConstants.MOTOR_CONFIGURATION.Slot0.kV,
        value -> {
          shooterRollerMotor1
              .getConfigurator()
              .apply(ShooterRollersConstants.MOTOR_CONFIGURATION.Slot0.withKV(value));
        });

    DogLog.tunable(
        "shooterRoller/kP",
        ShooterRollersConstants.MOTOR_CONFIGURATION.Slot0.kP,
        value -> {
          shooterRollerMotor1
              .getConfigurator()
              .apply(ShooterRollersConstants.MOTOR_CONFIGURATION.Slot0.withKP(value));
        });

    DogLog.tunable(
        "shooterRoller/kI",
        ShooterRollersConstants.MOTOR_CONFIGURATION.Slot0.kI,
        value -> {
          shooterRollerMotor1
              .getConfigurator()
              .apply(ShooterRollersConstants.MOTOR_CONFIGURATION.Slot0.withKI(value));
        });

    shooterRollerMotor2.setControl(
        new Follower(shooterRollerMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    if (RobotBase.isSimulation()) {
      simulation = new ShooterRollersSim(shooterRollerMotor1);
    }

    CurrentDrawLogger.add("Shooter Rollers", this::getSupplyCurrent);
  }

  /**
   * this makes the motor go, positive voltage makes CAN ID 21 go counter clockwise, which is not
   * what we want, please put a negative value so it goes clockwise since thats what is intended by
   * the build team
   */
  public Command shoot(AngularVelocity targetVelocity) {
    return shoot(() -> targetVelocity);
  }

  /**
   * Creates a command that drives the shooter roller to a dynamically supplied velocity while
   * scheduled.
   *
   * @param targetVelocity supplier that provides the desired velocity setpoint (in the units
   *     expected by VelocityVoltage)
   * @return a Command that, when scheduled, drives the shooter roller to the supplied velocity and
   *     coasts the motor on end
   */
  public Command shoot(Supplier<AngularVelocity> targetVelocitySupplier) {
    return runEnd(
        () -> {
          AngularVelocity targetVelocity = targetVelocitySupplier.get();

          if (targetVelocity == null) {
            targetVelocity = getAngularVelocity();
          }

          if (getAngularVelocity()
              .plus(ShooterRollersConstants.BANG_BANG_TOLERANCE)
              .lt(targetVelocity)) {
            shooterRollerMotor1.setControl(new DutyCycleOut(1).withEnableFOC(false));
          } else {
            // defines a local function to set motor voltage to make it go
            shooterRollerMotor1.setControl(
                new VelocityVoltage(targetVelocity.in(RotationsPerSecond)).withEnableFOC(false));
          }
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
        angleSignal,
        velocitySignal,
        voltageSignal,
        supplyCurrentSignal,
        otherSupplyCurrentSignal,
        statorCurrentSignal,
        accelerationSignal);
    DogLog.log("Shooter Rollers/voltage", getMotorVoltage());
    DogLog.log("Shooter Rollers/angularVelocity", getAngularVelocity());
    DogLog.log("Shooter Rollers/statorCurrent", getStatorCurrent());
    DogLog.log("Shooter Rollers/angularAcceleration", getAngularAcceleration());
    DogLog.log("Shooter Rollers/supplyCurrent", getSupplyCurrent());
  }

  /** gets the angular velocity */
  public AngularVelocity getAngularVelocity() {
    return velocitySignal.getValue();
  }

  /** gets the angular acceleration */
  public AngularAcceleration getAngularAcceleration() {
    return accelerationSignal.getValue();
  }

  /** gets the supply current */
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue().plus(otherSupplyCurrentSignal.getValue());
  }

  /** gets the stator current */
  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  /** gets the motor voltage */
  public Voltage getMotorVoltage() {
    return voltageSignal.getValue();
  }

  public Command sysId() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(2), Volts.of(7), Seconds.of(5)),
            new SysIdRoutine.Mechanism(
                voltage ->
                    shooterRollerMotor1.setControl(new VoltageOut(voltage).withEnableFOC(false)),
                log ->
                    log.motor("Shooter Rollers")
                        .angularPosition(angleSignal.getValue())
                        .angularVelocity(getAngularVelocity())
                        .angularAcceleration(getAngularAcceleration())
                        .voltage(getMotorVoltage()),
                this,
                "Shooter Rollers"));

    return Commands.sequence(
        routine.quasistatic(Direction.kForward),
        Commands.waitSeconds(5),
        routine.quasistatic(Direction.kReverse),
        Commands.waitSeconds(5),
        routine.dynamic(Direction.kForward),
        Commands.waitSeconds(5),
        routine.dynamic(Direction.kReverse));
  }
}
