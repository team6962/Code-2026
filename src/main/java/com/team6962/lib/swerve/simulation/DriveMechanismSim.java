package com.team6962.lib.swerve.simulation;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.math.WheelMath;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DriveMechanismSim {
    private Corner corner;
    private DrivetrainConstants constants;

    private TalonFXSimState motorControllerSimulation;
    private DCMotorSim physicsSimulation;

    public DriveMechanismSim(Corner corner, DrivetrainConstants constants, TalonFX motorController) {
        this.corner = corner;
        this.constants = constants;

        this.motorControllerSimulation = motorController.getSimState();

        this.physicsSimulation = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                constants.DriveMotor.SimulatedMotor,
                constants.DriveMotor.SimulatedMomentOfInertia.in(KilogramSquareMeters),
                constants.DriveMotor.GearReduction
            ),
            constants.DriveMotor.SimulatedMotor
        );
    }

    public Angle getAngularPosition() {
        return physicsSimulation.getAngularPosition();
    }

    public AngularVelocity getAngularVelocity() {
        return physicsSimulation.getAngularVelocity();
    }

    public AngularAcceleration getAngularAcceleration() {
        return physicsSimulation.getAngularAcceleration();
    }

    public Distance getPosition() {
        return WheelMath.toLinear(physicsSimulation.getAngularPosition(), constants.getWheelRadius(corner));
    }

    public LinearVelocity getVelocity() {
        return WheelMath.toLinear(physicsSimulation.getAngularVelocity(), constants.getWheelRadius(corner));
    }

    public LinearAcceleration getAcceleration() {
        return WheelMath.toLinear(physicsSimulation.getAngularAcceleration(), constants.getWheelRadius(corner));
    }

    public void update(double deltaTimeSeconds) {
        // Set the motor controller's supply voltage to the battery voltage
        // reported by RobotController, which can be overriden in simulation
        motorControllerSimulation.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Sign multiplier to account for motor inversion setting. The motor
        // controller reports values in its configured positive direction, but
        // the physics simulation always uses counter-clockwise-positive, so we
        // need to negate values when the motor is configured as
        // clockwise-positive.
        double motorSign = constants.DriveMotor.DeviceConfiguration.MotorOutput.Inverted == InvertedValue.Clockwise_Positive ? -1.0 : 1.0;

        // Set the physics simulation input voltage to the motor controller's
        // applied output voltage
        physicsSimulation.setInputVoltage(motorControllerSimulation.getMotorVoltage() * motorSign);

        // Update the physics simulation
        physicsSimulation.update(deltaTimeSeconds);

        // Update the motor controller simulation with the new position,
        // velocity, and acceleration from the physics simulation
        motorControllerSimulation.setRawRotorPosition(physicsSimulation.getAngularPosition().times(constants.DriveMotor.GearReduction * motorSign));
        motorControllerSimulation.setRotorVelocity(physicsSimulation.getAngularVelocity().times(constants.DriveMotor.GearReduction * motorSign));
        motorControllerSimulation.setRotorAcceleration(physicsSimulation.getAngularAcceleration().times(constants.DriveMotor.GearReduction * motorSign));

        DogLog.log("Drivetrain/Simulation/" + corner.name() + "Drive/Position", getPosition().in(Meters));
    }
}
