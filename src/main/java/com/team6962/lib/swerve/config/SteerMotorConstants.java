package com.team6962.lib.swerve.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6962.lib.swerve.config.control.ControlMode;

/**
 * The constants that define the steer motor behavior.
 */
public class SteerMotorConstants {
    /**
     * The TalonFX configuration for the steer motor. Some fields in this
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
    public TalonFXConfiguration DeviceConfiguration;

    /**
     * The gear reduction from the steer motor to the orientation of the wheel
     * about the steer axis.
     */
    public double GearReduction = 1.0;

    /**
     * Control mode for position control of the steer motor.
     */
    public ControlMode PositionControl = new ControlMode();

    /**
     * The slot index for position control of the steer motor. This should be
     * the index of the slot in the TalonFXConfiguration that has the position
     * PID and feedforward constants configured.
     */
    public int PositionSlot = 0;

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
}
