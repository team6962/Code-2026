package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team6962.lib.logging.CurrentDrawLogger;
import com.team6962.lib.phoenix.StatusUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
  private TalonFX leaderMotor;
  private TalonFX followerMotor;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Voltage> appliedVoltageSignal;
  private IntakeRollerSim simulation;
  private double intakeVoltage = 12.0;
  private double intakeStallVoltage = 12.0;
  private Debouncer stallDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private boolean stalling = false;

  /** Intializes motor and status signals Class for Intake Rollers */
  public IntakeRollers() {
    leaderMotor = new TalonFX(IntakeRollersConstants.LEADER_ID_1, IntakeRollersConstants.CANBUS);

    leaderMotor.getConfigurator().apply(IntakeRollersConstants.MOTOR_CONFIGURATION);

    followerMotor =
        new TalonFX(IntakeRollersConstants.FOLLOWER_ID_2, IntakeRollersConstants.CANBUS);

    IntakeRollersConstants.MOTOR_CONFIGURATION.MotorOutput.Inverted =
        IntakeRollersConstants.MOTOR_CONFIGURATION.MotorOutput.Inverted
                == InvertedValue.Clockwise_Positive
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    followerMotor.getConfigurator().apply(IntakeRollersConstants.MOTOR_CONFIGURATION);

    this.velocitySignal = leaderMotor.getVelocity();
    this.statorCurrentSignal = leaderMotor.getStatorCurrent();
    this.supplyCurrentSignal = followerMotor.getSupplyCurrent();
    this.appliedVoltageSignal = leaderMotor.getMotorVoltage();
    if (RobotBase.isSimulation()) {
      simulation = new IntakeRollerSim(leaderMotor);
    }

    DogLog.tunable(
        "Intake Voltage",
        intakeVoltage,
        value -> {
          intakeVoltage = value;
        });

    DogLog.tunable(
        "Intake Stall Voltage",
        intakeStallVoltage,
        value -> {
          intakeStallVoltage = value;
        });

    CurrentDrawLogger.add("Intake Rollers", this::getSupplyCurrent);

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    if (RobotBase.isSimulation()) {
      simulation = new IntakeRollerSim(leaderMotor);
    }
  }

  /** Returns command to make the motor move and stop */
  @SuppressWarnings("unused")
  private Command move(Voltage voltage) {
    return startEnd(
        () -> {
          leaderMotor.setControl(new VoltageOut(voltage).withEnableFOC(false));
        },
        () -> {
          leaderMotor.setControl(new CoastOut());
        });
  }

  /**
   * Returns command where motor intakes fuel
   *
   * @return Command
   */
  public Command intake() {
    return runEnd(
        () -> {
          leaderMotor.setControl(
              new VoltageOut(stalling ? intakeStallVoltage : intakeVoltage).withEnableFOC(false));
        },
        () -> {
          leaderMotor.setControl(new CoastOut());
        });
  }

  /**
   * Returns command where motor outtakes fuel
   *
   * @return
   */
  public Command outtake() {
    return startEnd(
        () -> {
          leaderMotor.setControl(new DutyCycleOut(-1).withEnableFOC(false));
        },
        () -> {
          leaderMotor.setControl(new CoastOut());
        });
  }

  /**
   * Takes status signal velocity and returns it as an AngularVelocity
   *
   * @return AngularVelocity
   */
  public AngularVelocity getVelocity() {
    return velocitySignal.getValue();
  }

  /**
   * Takes status signal Stator Current and returns it as a Current
   *
   * @return Current
   */
  public Current getStatorCurrent() {
    return statorCurrentSignal.getValue();
  }

  /**
   * Returns the total supply current draw of the mechanism.
   * It is multiplied by 2 to account for the second intake roller motor.
   *
   * @return Current
   */
  public Current getSupplyCurrent() {
    return Amps.of((2 * supplyCurrentSignal.getValueAsDouble()));
  }

  /**
   * Takes status signal Applied Voltage and returns it as a Voltage
   *
   * @return Voltage
   */
  public Voltage getAppliedVoltage() {
    return appliedVoltageSignal.getValue();
  }

  @Override
  public void periodic() {
    if (simulation != null) {
      simulation.update();
    }
    StatusUtil.check(
        BaseStatusSignal.refreshAll(
            velocitySignal, statorCurrentSignal, supplyCurrentSignal, appliedVoltageSignal));
    DogLog.log("intakeRollers/velocity", getVelocity());
    DogLog.log("intakeRollers/statorCurrent", getStatorCurrent());
    DogLog.log("intakeRollers/supplyCurrent", getSupplyCurrent());
    DogLog.log("intakeRollers/appliedVoltage", getAppliedVoltage());
    DogLog.log("intakeRollers/stalling", stalling);

    stalling =
        stallDebouncer.calculate(
            getVelocity().abs(RotationsPerSecond) < 1.0 && getStatorCurrent().abs(Amps) > 100.0);
  }
}
