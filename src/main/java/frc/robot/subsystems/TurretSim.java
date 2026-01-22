package frc.robot.subsystems;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretSim{
    private TalonFXSimState motorSim;
    // private AngularVelocity lastVelocity = RadiansPerSecond.of(0);
    private DCMotorSim physicsSim;
    public TurretSim (TalonFX motor) {
        motorSim = motor.getSimState();
        physicsSim = new DCMotorSim( 
            LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(1),
            0.000174, 42.0
            ),
        DCMotor.getKrakenX60Foc(1)
        );
    }
    public void update() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double motorVoltage = invert(motorSim.getMotorVoltage(),
            false);
        Angle position = physicsSim.getAngularPosition();
        AngularVelocity velocity = RadiansPerSecond.of(physicsSim.getAngularVelocityRadPerSec());
        physicsSim.setInput(motorVoltage);
        physicsSim.update(0.02);
        motorSim.setRawRotorPosition(
            invert(position, false)
            .times(42.0));
        motorSim.setRotorVelocity(
            invert(velocity, false)
            .times(42.0));
    }
    private static Angle invert(Angle angle, boolean shouldBeInverted) {
        return shouldBeInverted ? angle.unaryMinus() : angle;
    }
    private static AngularVelocity invert(AngularVelocity velocity, boolean shouldBeInverted) {
        return shouldBeInverted ? velocity.unaryMinus() : velocity;
    }
    public static double invert(double value, boolean shouldBeInverted) {
        return shouldBeInverted ? -value : value;
    }
};