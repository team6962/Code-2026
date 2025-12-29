package com.team6962.lib.swerve.config;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

/**
 * The constants that define the drive motor behavior.
 */
public class DriveMotorConstants {
    /**
     * The TalonFX configuration for the drive motor. Some fields in this
     * configuration may be overriden by other settings, such as the gear
     * reduction.
     * <p>
     * Some settings thay you may want to configure:
     * <ul>
     * <li>Inversion</li>
     * <li>Neutral Mode</li>
     * <li>Current Limits</li>
     * <li>Motion Magic</li>
     * <li>Slot Configs</li>
     * </ul>
     */
    public TalonFXConfiguration DeviceConfiguration = new TalonFXConfiguration();

    /**
     * The gear reduction from the drive motor to the wheel.
     */
    public double GearReduction = 1.0;

    /**
     * Control mode for position control of the drive motor.
     */
    public ControlMode.Position PositionControl = new ControlMode.Position();

    /**
     * The slot index for position control of the drive motor. This should be
     * the index of the slot in the TalonFXConfiguration that has the position
     * PID and feedforward constants configured.
     */
    public int PositionSlot = 0;

    /**
     * Control mode for velocity control of the drive motor.
     */
    public ControlMode.Velocity VelocityControl = new ControlMode.Velocity();

    /**
     * The slot index for velocity control of the drive motor. This should be
     * the index of the slot in the TalonFXConfiguration that has the velocity
     * PID and feedforward constants configured.
     */
    public int VelocitySlot = 1;

    /**
     * Control mode for dynamically-constrained position control of the drive motor.
     */
    public ControlMode.DynamicallyConstrainedPosition DynamicallyConstraintedPositionControl = new ControlMode.DynamicallyConstrainedPosition();

    /**
     * True if latency compensation should be performed on position data from
     * the motor's internal encoder. The latency compensation uses the velocity
     * data to extrapolate the position to account for the time delay between
     * when the position was measured and when the control loop runs.
     */
    public boolean PositionLatencyCompensation = true;

    /**
     * True if latency compensation should be performed on velocity data from
     * the motor's internal encoder. The latency compensation uses the
     * acceleration to extrapolate the velocity to account for the time delay
     * between when the velocity was measured and when the control loop runs.
     */
    public boolean VelocityLatencyCompensation = true;

    public DCMotor SimulatedMotor = DCMotor.getKrakenX60Foc(1);
    public MomentOfInertia SimulatedMomentOfInertia = KilogramSquareMeters.of(0.00032);
    public LinearVelocity MaxVelocity;

    /**
     * Constructs a DriveMotorConstants object with default values.
     */
    public DriveMotorConstants() {
    }

    /**
     * Sets the device configuration for the drive motor, and returns this
     * DriveMotorConstants for chaining.
     * 
     * @param deviceConfiguration The TalonFX configuration
     * @return                    This DriveMotorConstants object
     */
    public DriveMotorConstants withDeviceConfiguration(TalonFXConfiguration deviceConfiguration) {
        DeviceConfiguration = deviceConfiguration;
        return this;
    }

    /**
     * Sets the gear reduction for the drive motor, and returns this
     * DriveMotorConstants for chaining.
     * 
     * @param gearReduction The gear reduction
     * @return              This DriveMotorConstants object
     */
    public DriveMotorConstants withGearReduction(double gearReduction) {
        GearReduction = gearReduction;
        return this;
    }

    /**
     * Sets the position control mode for the drive motor, and returns this
     * DriveMotorConstants for chaining.
     * 
     * @param positionControl The position control mode
     * @return                This DriveMotorConstants object
     */
    public DriveMotorConstants withPositionControl(ControlMode.Position positionControl) {
        PositionControl = positionControl;
        return this;
    }

    /**
     * Sets the position slot index for the drive motor, and returns this
     * DriveMotorConstants for chaining.
     * 
     * @param positionSlot The position slot index
     * @return             This DriveMotorConstants object
     */
    public DriveMotorConstants withPositionSlot(int positionSlot) {
        PositionSlot = positionSlot;
        return this;
    }

    /**
     * Sets the velocity control mode for the drive motor, and returns this
     * DriveMotorConstants for chaining.
     * 
     * @param velocityControl The velocity control mode
     * @return                This DriveMotorConstants object
     */
    public DriveMotorConstants withVelocityControl(ControlMode.Velocity velocityControl) {
        VelocityControl = velocityControl;
        return this;
    }

    /**
     * Sets the velocity slot index for the drive motor, and returns this
     * DriveMotorConstants for chaining.
     * 
     * @param velocitySlot The velocity slot index
     * @return             This DriveMotorConstants object
     */
    public DriveMotorConstants withVelocitySlot(int velocitySlot) {
        VelocitySlot = velocitySlot;
        return this;
    }

    /**
     * Sets the dynamically-constrained position control mode for the drive
     * motor, and returns this DriveMotorConstants for chaining.
     * 
     * @param dynamicallyConstraintedPositionControl The dynamically-constrained
     *                                               position control mode
     * @return                                       This DriveMotorConstants
     *                                               object
     */
    public DriveMotorConstants withDynamicallyConstraintedPositionControl(
            ControlMode.DynamicallyConstrainedPosition dynamicallyConstraintedPositionControl) {
        DynamicallyConstraintedPositionControl = dynamicallyConstraintedPositionControl;
        return this;
    }

    /**
     * Sets whether position latency compensation is enabled, and returns this
     * DriveMotorConstants for chaining.
     * 
     * @param positionLatencyCompensation True if enabled
     * @return                            This DriveMotorConstants object
     */
    public DriveMotorConstants withPositionLatencyCompensation(boolean positionLatencyCompensation) {
        PositionLatencyCompensation = positionLatencyCompensation;
        return this;
    }

    /**
     * Sets whether velocity latency compensation is enabled, and returns this
     * DriveMotorConstants for chaining.
     * 
     * @param velocityLatencyCompensation True if enabled
     * @return                            This DriveMotorConstants object
     */
    public DriveMotorConstants withVelocityLatencyCompensation(boolean velocityLatencyCompensation) {
        VelocityLatencyCompensation = velocityLatencyCompensation;
        return this;
    }

    public DriveMotorConstants withSimulatedMotor(DCMotor simulatedMotor) {
        SimulatedMotor = simulatedMotor;
        return this;
    }

    public DriveMotorConstants withSimulatedMomentOfInertia(MomentOfInertia simulatedMomentOfInertia) {
        SimulatedMomentOfInertia = simulatedMomentOfInertia;
        return this;
    }

    public DriveMotorConstants withMaxVelocity(LinearVelocity maxVelocity) {
        MaxVelocity = maxVelocity;
        return this;
    }
}
