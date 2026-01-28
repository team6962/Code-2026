package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class defines the status, velocity, voltage, and current of the intake subsystem
 */
public class IntakeRollers extends SubsystemBase {

  private TalonFX intakeMotor;
  private StatusSignal<AngularVelocity> intakeVelocity;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<Voltage> appliedVoltageSignal;
//private double RPM;
    /**
     * Initializes the motor and status signal
     */
  public IntakeRollers() {
//constructer --> initiallize
    this.intakeMotor = new TalonFX(1, new CANBus("canivore")); //temporary
    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.CurrentLimits.StatorCurrentLimitEnable = false;
    configuration.CurrentLimits.SupplyCurrentLimitEnable = false;
    configuration.Slot0.kV = 0.12;
    intakeMotor.getConfigurator().apply(configuration);

    this.intakeVelocity = intakeMotor.getVelocity();
    this.statorCurrentSignal = intakeMotor.getStatorCurrent();
    this.supplyCurrentSignal = intakeMotor.getSupplyCurrent();
    this.appliedVoltageSignal = intakeMotor.getMotorVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    BaseStatusSignal.refreshAll(statorCurrentSignal, supplyCurrentSignal, appliedVoltageSignal, intakeVelocity);
    DogLog.log("Intake Rollers/Velocity", getIntakeVelocity());
    DogLog.log("Intake Rollers/StatorCurrent", getStatorCurrentSignal());
    DogLog.log("Intake Rollers/Supply Current", getSupplyCurrentSignal());
    DogLog.log("Intake Rollers/Motor Voltage", getAppliedVoltageSignal());
  }

  public Command move(){
    return startEnd(() -> {
        intakeMotor.setControl(new VelocityVoltage(60));
    },() -> {
        intakeMotor.setControl(new CoastOut());
    });
  }

  /**
   * Gets value of angular velocity of the intake rollers
   */
  public AngularVelocity getIntakeVelocity(){
    return intakeVelocity.getValue();
  }

  public Current getStatorCurrentSignal(){
    return statorCurrentSignal.getValue();
  }

  public Current getSupplyCurrentSignal(){
    return supplyCurrentSignal.getValue();
  }

  public Voltage getAppliedVoltageSignal(){
    return appliedVoltageSignal.getValue();
  }
}