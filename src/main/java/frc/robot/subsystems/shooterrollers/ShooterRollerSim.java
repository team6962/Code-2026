package frc.robot.subsystems.shooterrollers;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterRollerSim {
    private TalonFXSimState motorSim;
    private DCMotorSim physicsSim;
    public ShooterRollerSim(TalonFX motor) {
        motorSim = motor.getSimState();
        physicsSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60Foc(2), 
        //note: ask build what motor will be used
        0.000174,0.75),
            DCMotor.getKrakenX60Foc(2)    
        
        );
    }
    public void update() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double motorVoltage = invert(motorSim.getMotorVoltage(), false);
        Angle position = physicsSim.getAngularPosition();
        AngularVelocity velocity = RadiansPerSecond.of(physicsSim.getAngularVelocityRadPerSec());
        physicsSim.setInput(motorVoltage);
        physicsSim.update(0.02);
        motorSim.setRawRotorPosition(
            invert(position,false)
            .times(0.75)
        );
        motorSim.setRotorVelocity(
            invert(velocity,false)
            .times(0.75)
        );

    }

    private static Angle invert(Angle angle, boolean shouldBeInverted) {
        return shouldBeInverted ? angle.unaryMinus() : angle;
    }
    private static AngularVelocity invert(AngularVelocity velocity, boolean shouldBeInverted) {
        return shouldBeInverted ? velocity.unaryMinus() : velocity;
    }
    private static double invert(double value, boolean shouldBeInverted) {
        return shouldBeInverted ? -value : value;
    }
}
