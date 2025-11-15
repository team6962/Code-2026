package com.team6962.lib.swerve.config;

import com.ctre.phoenix6.configs.CANcoderConfiguration;

/**
 * The constants that define the steer encoder behavior.
 */
public class SteerEncoderConstants {
    /**
     * The CANcoder configuration for the steer encoder. Some fields in this
     * configuration may be overriden by other setting.
     * <p>
     * It's very unlikely thay you'll need to modify this, but perhaps new
     * configurable features will be added in the future. Currently, the only
     * possibly useful value that is not set elsewhere is the sensor direction.
     */
    public CANcoderConfiguration DeviceConfiguration = new CANcoderConfiguration();

    /**
     * Constructs a SteerEncoderConstants object with default values.
     */
    public SteerEncoderConstants() {
        DeviceConfiguration = new CANcoderConfiguration();
    }

    /**
     * Sets the device configuration for the steer encoder, and returns this
     * SteerEncoderConstants for chaining.
     * 
     * @param deviceConfiguration The CANcoder configuration
     * @return                    This SteerEncoderConstants object
     */
    public SteerEncoderConstants withDeviceConfiguration(CANcoderConfiguration deviceConfiguration) {
        DeviceConfiguration = deviceConfiguration;
        return this;
    }
}
