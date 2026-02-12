package frc.robot.subsystems.hopper.kicker;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intakerollers.IntakeRollersConstants;

public class KickerSim {
    private TalonFXSimState kickerMotor;
    private DCMotorSim physicsSim;

    public KickerSim(TalonFX motor) {
        kickerMotor = motor.getSimState();
        physicsSim = 
            new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1),
                HopperConstants.KICKER_MOMENT_OF_INERTIA,
                HopperConstants.KICKER_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(1));
    }
    

    public void update() {
        kickerMotor.setSupplyVoltage(12);
        physicsSim.setInputVoltage(kickerMotor.getMotorVoltage());
        physicsSim.update(0.02);

        kickerMotor.setRawRotorPosition(
            physicsSim.getAngularPosition().times(HopperConstants.KICKER_GEAR_RATIO));
        kickerMotor.setRotorVelocity(
            physicsSim.getAngularVelocity().times(HopperConstants.KICKER_GEAR_RATIO));
        kickerMotor.setRotorAcceleration(
            physicsSim.getAngularAcceleration().times(HopperConstants.KICKER_GEAR_RATIO));

    }

}