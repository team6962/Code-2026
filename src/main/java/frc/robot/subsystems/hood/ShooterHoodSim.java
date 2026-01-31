package frc.robot.subsystems.hood;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterHoodSim {
    private TalonFXSimState motorSim;
    private AngularVelocity lastVelocity = RadiansPerSecond.of(0);
    private SingleJointedArmSim physicsSim; 
    public ShooterHoodSim(TalonFX motor) {
        motorSim = motor.getSimState();
        physicsSim = new SingleJointedArmSim(
            ShooterHoodConstants.MOTOR_PHYSICS,
            ShooterHoodConstants.MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio,
            ShooterHoodConstants.MOI,
            Inches.of(6.85).in(Meters),
            ShooterHoodConstants.MIN_ANGLE.in(Radians),
            ShooterHoodConstants.MAX_ANGLE.in(Radians),
            true,
            0
        );
    }

    public void update() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double motorVoltage = invert(motorSim.getMotorVoltage(),
            false
        );

        physicsSim.setInput(motorVoltage);
        physicsSim.update(0.02);

        Angle position = Radians.of(physicsSim.getAngleRads());
        AngularVelocity velocity = RadiansPerSecond.of(physicsSim.getVelocityRadPerSec());
        AngularAcceleration acceleration = velocity.minus(lastVelocity).div(Seconds.of(0.02));

        motorSim.setRawRotorPosition(
            invert(position, false)
                .times(ShooterHoodConstants.MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio)
        );

        motorSim.setRotorVelocity(
            invert(velocity, false)
                .times(ShooterHoodConstants.MOTOR_CONFIGURATION.Feedback.SensorToMechanismRatio)
        );
    }

    private static double invert(double value, boolean shouldBeInverted) {
        return shouldBeInverted ? -value : value;
    }

    private static Angle invert(Angle angle, boolean shouldBeInverted) {
        return shouldBeInverted ? angle.unaryMinus() : angle;
    }

    private static AngularVelocity invert(AngularVelocity velocity, boolean shouldBeInverted) {
        return shouldBeInverted ? velocity.unaryMinus() : velocity;
    }
}   
