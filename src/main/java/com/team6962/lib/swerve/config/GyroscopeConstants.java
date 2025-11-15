package com.team6962.lib.swerve.config;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

/**
 * The constants that define the behavior of the gyroscope used for determining
 * the heading of the robot.
 */
public class GyroscopeConstants {
    /**
     * The CAN ID of the Pigeon 2 gyroscope.
     */
    public int CANId = -1;

    /**
     * The Pigeon 2 configuration. Some fields in this configuration may be
     * overriden by other settings.
     */
    public Pigeon2Configuration DeviceConfiguration = new Pigeon2Configuration();

    /**
     * If false, the gyroscope will be disabled and only the wheel encoder
     * odometry will be used. This value should never be false for competition
     * matches in normal conditions. It's primary use is for running a robot
     * that is placed on a stand or cart so it doesn't actually move, which is a
     * good way to test an autonomous routine in the pit.
     */
    public boolean Enabled = true;

    /**
     * Constructs a GyroscopeConstants object with default values.
     */
    public GyroscopeConstants() {
        CANId = -1;
        DeviceConfiguration = new Pigeon2Configuration();
        Enabled = true;
    }

    /**
     * Sets the CAN ID of the gyroscope, and returns this GyroscopeConstants
     * for chaining.
     * 
     * @param canId The CAN ID
     * @return      This GyroscopeConstants object
     */
    public GyroscopeConstants withCANId(int canId) {
        CANId = canId;
        return this;
    }

    /**
     * Sets the device configuration for the gyroscope, and returns this
     * GyroscopeConstants for chaining.
     * 
     * @param deviceConfiguration The Pigeon2 configuration
     * @return                    This GyroscopeConstants object
     */
    public GyroscopeConstants withDeviceConfiguration(Pigeon2Configuration deviceConfiguration) {
        DeviceConfiguration = deviceConfiguration;
        return this;
    }

    /**
     * Sets whether the gyroscope is enabled, and returns this GyroscopeConstants
     * for chaining.
     * 
     * @param enabled True if the gyroscope is enabled
     * @return        This GyroscopeConstants object
     */
    public GyroscopeConstants withEnabled(boolean enabled) {
        Enabled = enabled;
        return this;
    }
}
