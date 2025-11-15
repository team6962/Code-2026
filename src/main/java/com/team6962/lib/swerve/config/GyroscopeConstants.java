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
}
