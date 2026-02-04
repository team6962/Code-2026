package frc.robot.subsystems.intakerollers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeRollerSim {
  private TalonFXSimState intakeMotor;
  private DCMotorSim physicsSim;

  /**
   * simulates physics on given motor
   *
   * @param motor (DCMotor)
   */
  public IntakeRollerSim(TalonFX motor) {
    intakeMotor = motor.getSimState();
    physicsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 5),
            DCMotor.getKrakenX60(1));
  }

  /** updates physicsSim */
  public void update() {
    intakeMotor.setSupplyVoltage(12);
    physicsSim.setInputVoltage(intakeMotor.getMotorVoltage());
    physicsSim.update(0.02);

    intakeMotor.setRawRotorPosition(physicsSim.getAngularPosition().times(5));
    intakeMotor.setRotorVelocity(physicsSim.getAngularVelocity().times(5));
    intakeMotor.setRotorAcceleration(physicsSim.getAngularAcceleration().times(5));
  }
}
