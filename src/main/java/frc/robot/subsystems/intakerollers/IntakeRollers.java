package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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
  private TalonFX intakeMotor;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Voltage> appliedVoltageSignal;
  private IntakeRollerSim simulation;
  private double intakeVoltage = 4.0;
  private double intakeStallVoltage = 12.0;
  private Debouncer stallDebouncer = new Debouncer(0.25, DebounceType.kRising);
  private boolean stalling = false;

  /** Intializes motor and status signals Class for Intake Rollers */
  public IntakeRollers() {
    this.intakeMotor =
        new TalonFX(IntakeRollersConstants.DEVICE_ID, new CANBus("subsystems")); // temporary

    intakeMotor.getConfigurator().apply(IntakeRollersConstants.MOTOR_CONFIGURATION);
    this.velocitySignal = intakeMotor.getVelocity();
    this.statorCurrentSignal = intakeMotor.getStatorCurrent();
    this.supplyCurrentSignal = intakeMotor.getSupplyCurrent();
    this.appliedVoltageSignal = intakeMotor.getMotorVoltage();
    if (RobotBase.isSimulation()) {
      simulation = new IntakeRollerSim(intakeMotor);
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
  }

  /** Returns command to make the motor move and stop */
  @SuppressWarnings("unused")
  private Command move(Voltage voltage) {
    return startEnd(
        () -> {
          intakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(false));
        },
        () -> {
          intakeMotor.setControl(new CoastOut());
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
          intakeMotor.setControl(
              new VoltageOut(stalling ? intakeStallVoltage : intakeVoltage).withEnableFOC(false));
        },
        () -> {
          intakeMotor.setControl(new CoastOut());
        });
  }

  /**
   * Returns command where motor intakes fuel at full speed. This should only be used when shooting
   * fuel out of the robot with the intake is not a concern.
   *
   * @return The command to intake fuel at full speed.
   */
  public Command intakeFast() {
    return startEnd(
        () -> {
          intakeMotor.setControl(new VoltageOut(7).withEnableFOC(false));
        },
        () -> {
          intakeMotor.setControl(new CoastOut());
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
          intakeMotor.setControl(new DutyCycleOut(-1).withEnableFOC(false));
        },
        () -> {
          intakeMotor.setControl(new CoastOut());
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
   * Takes status signal Supply Current and returns it as a Current
   *
   * @return Current
   */
  public Current getSupplyCurrent() {
    return supplyCurrentSignal.getValue();
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
