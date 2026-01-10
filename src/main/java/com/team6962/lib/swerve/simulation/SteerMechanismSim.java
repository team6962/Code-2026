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

/**
 * Simulates a swerve module's steer mechanism using WPILib's DCMotorSim
 * physics simulation.
 *
 * <p>This class bridges between CTRE's TalonFX and CANcoder simulation
 * states and WPILib's physics simulation. It reads the motor controller's
 * applied voltage, runs the physics simulation, and updates both the motor
 * controller's and encoder's simulated sensor values.
 *
 * <p>The simulation handles device inversion by converting between each
 * device's configured positive direction and the physics simulation's
 * counter-clockwise-positive convention. The motor and encoder may have
 * independent inversion settings.
 *
 * <p>Getter methods return the module's angular position, velocity, and
 * acceleration in mechanism (output shaft) coordinates.
 *
 * <p>The {@link #update(double)} method should be called periodically
 * (typically every simulation tick) to advance the physics simulation and
 * synchronize state with the motor controller and encoder.
 */
public class SteerMechanismSim {
    /** The corner position of this module on the robot. */
    private Corner corner;
    /** Drivetrain configuration containing motor parameters. */
    private DrivetrainConstants constants;

    /** Simulation state of the TalonFX motor controller. */
    private TalonFXSimState motorControllerSimulation;
    /** Simulation state of the CANcoder encoder. */
    private CANcoderSimState encoderSimulation;
    /** WPILib physics simulation for the DC motor. */
    private DCMotorSim physicsSimulation;

    /**
     * Creates a new steer mechanism simulation.
     *
     * @param corner the corner position of the module
     * @param constants drivetrain configuration
     * @param motorController the TalonFX motor controller to simulate
     * @param encoder the CANcoder encoder to simulate
     */
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

    /** Returns the angular position of the mechanism (output shaft). */
    public Angle getAngularPosition() {
        return physicsSimulation.getAngularPosition();
    }

    /** Returns the angular velocity of the mechanism (output shaft). */
    public AngularVelocity getAngularVelocity() {
        return physicsSimulation.getAngularVelocity();
    }

    /** Returns the angular acceleration of the mechanism (output shaft). */
    public AngularAcceleration getAngularAcceleration() {
        return physicsSimulation.getAngularAcceleration();
    }

    /**
     * Updates the physics simulation and synchronizes motor controller and encoder state.
     *
     * @param deltaTimeSeconds time elapsed since the last update
     */
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

        // The configured steer encoder offset, in mechanism coordinates
        Angle offset = constants.getSwerveModule(corner).SteerEncoderOffset.minus(corner.getRotation());

        // Set the physics simulation input voltage to the motor controller's
        // applied output voltage
        physicsSimulation.setInputVoltage(motorControllerSimulation.getMotorVoltage() * motorSign);

        // Update the physics simulation
        physicsSimulation.update(deltaTimeSeconds);

        // Update the motor controller simulation with the new position,
        // velocity, and acceleration from the physics simulation
        motorControllerSimulation.setRawRotorPosition(physicsSimulation.getAngularPosition().minus(offset).times(constants.SteerMotor.GearReduction * motorSign));
        motorControllerSimulation.setRotorVelocity(physicsSimulation.getAngularVelocity().times(constants.SteerMotor.GearReduction * motorSign));
        motorControllerSimulation.setRotorAcceleration(physicsSimulation.getAngularAcceleration().times(constants.SteerMotor.GearReduction * motorSign));

        // Update the encoder simulation with the new position and velocity from
        // the physics simulation
        encoderSimulation.setRawPosition(physicsSimulation.getAngularPosition().minus(offset).times(encoderSign));
        encoderSimulation.setVelocity(physicsSimulation.getAngularVelocity().times(encoderSign));

        DogLog.log("Drivetrain/Simulation/" + corner.name() + "Steer/Position", getAngularPosition().in(Radians));
    }
}
