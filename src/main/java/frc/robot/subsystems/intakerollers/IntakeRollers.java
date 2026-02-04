package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.phoenix.StatusUtil;
import dev.doglog.DogLog;
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

  /** Intializes motor and status signals Class for Intake Rollers */
  public IntakeRollers() {
    this.intakeMotor = new TalonFX(41, new CANBus("subsystem")); // temporary
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.Feedback.SensorToMechanismRatio = 5;
    configuration.CurrentLimits.StatorCurrentLimitEnable = true;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
    configuration.CurrentLimits.StatorCurrentLimit = 120;
    configuration.CurrentLimits.SupplyCurrentLimit = 60;
    intakeMotor.getConfigurator().apply(configuration);
    this.velocitySignal = intakeMotor.getVelocity();
    this.statorCurrentSignal = intakeMotor.getStatorCurrent();
    this.supplyCurrentSignal = intakeMotor.getSupplyCurrent();
    this.appliedVoltageSignal = intakeMotor.getMotorVoltage();
    if (RobotBase.isSimulation()) {
      simulation = new IntakeRollerSim(intakeMotor);
    }
  }

  /** Returns command to make the motor move and stop */
  private Command move(Voltage voltage) {
    return startEnd(
        () -> {
          intakeMotor.setControl(new VoltageOut(voltage));
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
    return move(Volts.of(12));
  }

  /**
   * Returns command where motor outtakes fuel
   *
   * @return
   */
  public Command outtake() {
    return move(Volts.of(-12));
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
  }
}
