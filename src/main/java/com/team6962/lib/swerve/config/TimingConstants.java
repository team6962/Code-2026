package com.team6962.lib.swerve.config;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Frequency;

/**
 * The constants that define various update frequencies and whether to use
 * timesync for control requests.
 */
public class TimingConstants {
    /**
     * The frequency at which devices should send status signal updates to the
     * robot controller.
     */
    public Frequency StatusSignalUpdateRate = Hertz.of(100);

    /**
     * The frequency at which to run the internal swerve control loop. Only
     * 50 Hz is supported at this time, so this field is ignored.
     */
    public Frequency ControlLoopFrequency = Hertz.of(50);

    /**
     * Whether to use time synchronization for control requests. Enabling this
     * will increase the delay between sending a control request and the motors
     * responding, but will make all motors respond simultaneously, which may
     * improve accuracy.
     */
    public boolean TimesyncControlRequests = false;
}
