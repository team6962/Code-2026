package com.team6962.lib.swerve.simulation;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.module.SwerveModuleConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SteerMotorSim {
    private SwerveModuleConfig moduleConfig;

    private TalonFXSimState motorControllerSimulation;
    private CANcoderSimState encoderSimulation;
    private DCMotorSim physicsSimulation;

    public SteerMotorSim(SwerveModuleConfig moduleConfig, SimulationConfig simConfig, TalonFX motorController, CANcoder encoder) {
        this.moduleConfig = moduleConfig;
        this.motorControllerSimulation = motorController.getSimState();
        this.encoderSimulation = encoder.getSimState();
        this.physicsSimulation = new DCMotorSim(simConfig.steerSimulationPlant, simConfig.steerSimulationGearbox);
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

    public void update(double deltaTimeSeconds) {
        // Set the device simulations' supply voltages to the battery voltage
        // reported by RobotController, which can be overriden in simulation
        motorControllerSimulation.setSupplyVoltage(RobotController.getBatteryVoltage());
        encoderSimulation.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Set the physics simulation input voltage to the motor controller's
        // applied output voltage
        physicsSimulation.setInputVoltage(motorControllerSimulation.getMotorVoltage());

        // Update the physics simulation
        physicsSimulation.update(deltaTimeSeconds);

        // Update the motor controller simulation with the new position,
        // velocity, and acceleration from the physics simulation
        motorControllerSimulation.setRawRotorPosition(physicsSimulation.getAngularPosition().times(moduleConfig.steerMotorConfiguration.Feedback.RotorToSensorRatio));
        motorControllerSimulation.setRotorVelocity(physicsSimulation.getAngularVelocity().times(moduleConfig.steerMotorConfiguration.Feedback.RotorToSensorRatio));
        motorControllerSimulation.setRotorAcceleration(physicsSimulation.getAngularAcceleration().times(moduleConfig.steerMotorConfiguration.Feedback.RotorToSensorRatio));

        // Update the encoder simulation with the new position and velocity from
        // the physics simulation
        encoderSimulation.setRawPosition(physicsSimulation.getAngularPosition());
        encoderSimulation.setVelocity(physicsSimulation.getAngularVelocity());
    }
}
