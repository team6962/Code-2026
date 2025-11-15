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
    public Frequency SignalUpdateRate = Hertz.of(100);

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

    /**
     * Constructs a TimingConstants object with default values.
     */
    public TimingConstants() {
        SignalUpdateRate = Hertz.of(100);
        ControlLoopFrequency = Hertz.of(50);
        TimesyncControlRequests = false;
    }

    /**
     * Sets the status signal update rate, and returns this TimingConstants
     * for chaining.
     * 
     * @param signalUpdateRate The status signal update rate
     * @return                 This TimingConstants object
     */
    public TimingConstants withSignalUpdateRate(Frequency signalUpdateRate) {
        SignalUpdateRate = signalUpdateRate;
        return this;
    }

    /**
     * Sets the control loop frequency, and returns this TimingConstants
     * for chaining.
     * 
     * @param controlLoopFrequency The control loop frequency
     * @return                     This TimingConstants object
     */
    public TimingConstants withControlLoopFrequency(Frequency controlLoopFrequency) {
        ControlLoopFrequency = controlLoopFrequency;
        return this;
    }

    /**
     * Sets whether to use timesync for control requests, and returns this
     * TimingConstants for chaining.
     * 
     * @param timesyncControlRequests True if timesync is enabled
     * @return                        This TimingConstants object
     */
    public TimingConstants withTimesyncControlRequests(boolean timesyncControlRequests) {
        TimesyncControlRequests = timesyncControlRequests;
        return this;
    }
}
