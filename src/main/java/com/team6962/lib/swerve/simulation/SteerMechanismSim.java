package com.team6962.lib.swerve.simulation;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SteerMechanismSim {
    private Corner corner;
    private DrivetrainConstants constants;

    private TalonFXSimState motorControllerSimulation;
    private CANcoderSimState encoderSimulation;
    private DCMotorSim physicsSimulation;

    public SteerMechanismSim(Corner corner, DrivetrainConstants constants, TalonFX motorController, CANcoder encoder) {
        this.corner = corner;
        this.constants = constants;

        this.motorControllerSimulation = motorController.getSimState();
        this.encoderSimulation = encoder.getSimState();

        this.physicsSimulation = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                constants.SteerMotor.SimulatedMotor,
                constants.SteerMotor.SimulatedMomentOfInertia.in(KilogramSquareMeters),
                constants.SteerMotor.GearReduction
            ),
            constants.SteerMotor.SimulatedMotor
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

    public void update(double deltaTimeSeconds) {
        // Set the device simulations' supply voltages to the battery voltage
        // reported by RobotController, which can be overriden in simulation
        motorControllerSimulation.setSupplyVoltage(RobotController.getBatteryVoltage());
        encoderSimulation.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Sign multipliers to account for device inversion settings. The motor
        // controller and encoder report values in their configured positive
        // direction, but the physics simulation always uses
        // counter-clockwise-positive, so we need to negate values when the
        // devices are configured as clockwise-positive.
        double motorSign = constants.SteerMotor.DeviceConfiguration.MotorOutput.Inverted == InvertedValue.Clockwise_Positive ? -1.0 : 1.0;
        double encoderSign = constants.SteerEncoder.DeviceConfiguration.MagnetSensor.SensorDirection == SensorDirectionValue.Clockwise_Positive ? -1.0 : 1.0;

        // Set the physics simulation input voltage to the motor controller's
        // applied output voltage
        physicsSimulation.setInputVoltage(motorControllerSimulation.getMotorVoltage() * motorSign);

        // Update the physics simulation
        physicsSimulation.update(deltaTimeSeconds);

        // Update the motor controller simulation with the new position,
        // velocity, and acceleration from the physics simulation
        motorControllerSimulation.setRawRotorPosition(physicsSimulation.getAngularPosition().times(constants.SteerMotor.GearReduction * motorSign));
        motorControllerSimulation.setRotorVelocity(physicsSimulation.getAngularVelocity().times(constants.SteerMotor.GearReduction * motorSign));
        motorControllerSimulation.setRotorAcceleration(physicsSimulation.getAngularAcceleration().times(constants.SteerMotor.GearReduction * motorSign));

        // Update the encoder simulation with the new position and velocity from
        // the physics simulation
        encoderSimulation.setRawPosition(physicsSimulation.getAngularPosition().times(encoderSign));
        encoderSimulation.setVelocity(physicsSimulation.getAngularVelocity().times(encoderSign));

        DogLog.log("Drivetrain/Simulation/" + corner.name() + "Steer/Position", getAngularPosition().in(Radians));
    }
}
