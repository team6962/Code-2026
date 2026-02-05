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
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), IntakeRollersConstants.MOMENT_OF_INERTIA, IntakeRollersConstants.GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));
  }

  /** updates physicsSim */
  public void update() {
    intakeMotor.setSupplyVoltage(12);
    physicsSim.setInputVoltage(intakeMotor.getMotorVoltage());
    physicsSim.update(0.02);

    intakeMotor.setRawRotorPosition(physicsSim.getAngularPosition().times(IntakeRollersConstants.GEAR_RATIO));
    intakeMotor.setRotorVelocity(physicsSim.getAngularVelocity().times(IntakeRollersConstants.GEAR_RATIO));
    intakeMotor.setRotorAcceleration(physicsSim.getAngularAcceleration().times(IntakeRollersConstants.GEAR_RATIO));
  }
}
