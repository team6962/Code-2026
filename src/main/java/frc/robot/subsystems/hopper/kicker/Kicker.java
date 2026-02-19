package frc.robot.subsystems.hopper.kicker;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
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
import frc.robot.subsystems.hopper.HopperConstants;

/** The kicker class is responsible for controlling the kicker */
public class Kicker extends SubsystemBase {
  private TalonFX kickerMotor;
  private StatusSignal<Voltage> appliedVoltageSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Current> supplyCurrentSignal;
  private StatusSignal<AngularVelocity> motorVelocitySignal;
  private KickerSim simulation;

  public Kicker() {

    this.kickerMotor =
        new TalonFX(HopperConstants.KICKER_DEVICE_ID, new CANBus(HopperConstants.CANBUS_NAME));
    kickerMotor.getConfigurator().apply(HopperConstants.KICKER_MOTOR_CONFIGURATION);

    motorVelocitySignal = kickerMotor.getVelocity();
    statorCurrentSignal = kickerMotor.getStatorCurrent();
    supplyCurrentSignal = kickerMotor.getSupplyCurrent();
    appliedVoltageSignal = kickerMotor.getMotorVoltage();
    
    if (RobotBase.isSimulation()) {
      simulation = new KickerSim(kickerMotor);
    }
  }

  /**
   * Returns command to make the motor move and stop
   *
   * @param voltage target voltage
   * @return Command that runs at target voltage
   */
  private Command move(Voltage voltage) {
    return startEnd(
        () -> {
          kickerMotor.setControl(new VoltageOut(voltage));
        },
        () -> {
          kickerMotor.setControl(new NeutralOut());
        });
  }

  /**
   * feeds fuel to the shooter return
   *
   * @return command that feeds fuel
   */
  public Command feed() {
    return move(Volts.of(12)); // negative or positive to be tuned
  }

  /**
   * passes fuel back to intake
   *
   * @return command that passes fuel back
   */
  public Command reverse() {
    return move(Volts.of(-12)); // negative or positive to be tuned
  }

  /**
   * Takes status signal velocity and returns it as an AngularVelocity
   *
   * @return AngularVelocity
   */
  public AngularVelocity getVelocity() {
    return motorVelocitySignal.getValue();
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
            motorVelocitySignal, statorCurrentSignal, supplyCurrentSignal, appliedVoltageSignal));
    DogLog.log("Kicker/AngularVelocity", getVelocity());
    DogLog.log("Kicker/StatorCurrent", getStatorCurrent());
    DogLog.log("Kicker/SupplyCurrent", getSupplyCurrent());
    DogLog.log("Kicker/AppliedVoltage", getAppliedVoltage());
  }
}
