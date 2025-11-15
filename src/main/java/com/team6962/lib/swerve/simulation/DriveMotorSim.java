package com.team6962.lib.swerve.simulation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.math.WheelMath;
import com.team6962.lib.swerve.module.SwerveModuleConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DriveMotorSim {
    private SwerveModuleConfig moduleConfig;

    private TalonFXSimState motorControllerSimulation;
    private DCMotorSim physicsSimulation;

    public DriveMotorSim(SwerveModuleConfig moduleConfig, SimulationConfig simConfig, TalonFX motorController) {
        this.moduleConfig = moduleConfig;
        this.motorControllerSimulation = motorController.getSimState();
        this.physicsSimulation = new DCMotorSim(simConfig.driveSimulationPlant, simConfig.driveSimulationGearbox);
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
        return WheelMath.toLinear(physicsSimulation.getAngularPosition(), moduleConfig.wheelRadius);
    }

    public LinearVelocity getVelocity() {
        return WheelMath.toLinear(physicsSimulation.getAngularVelocity(), moduleConfig.wheelRadius);
    }

    public LinearAcceleration getAcceleration() {
        return WheelMath.toLinear(physicsSimulation.getAngularAcceleration(), moduleConfig.wheelRadius);
    }

    public void update(double deltaTimeSeconds) {
        // Set the motor controller's supply voltage to the battery voltage
        // reported by RobotController, which can be overriden in simulation
        motorControllerSimulation.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Set the physics simulation input voltage to the motor controller's
        // applied output voltage
        physicsSimulation.setInputVoltage(motorControllerSimulation.getMotorVoltage());

        // Update the physics simulation
        physicsSimulation.update(deltaTimeSeconds);

        // Update the motor controller simulation with the new position,
        // velocity, and acceleration from the physics simulation
        motorControllerSimulation.setRawRotorPosition(physicsSimulation.getAngularPosition().times(moduleConfig.driveMotorConfiguration.Feedback.RotorToSensorRatio));
        motorControllerSimulation.setRotorVelocity(physicsSimulation.getAngularVelocity().times(moduleConfig.driveMotorConfiguration.Feedback.RotorToSensorRatio));
        motorControllerSimulation.setRotorAcceleration(physicsSimulation.getAngularAcceleration().times(moduleConfig.driveMotorConfiguration.Feedback.RotorToSensorRatio));
    }
}
