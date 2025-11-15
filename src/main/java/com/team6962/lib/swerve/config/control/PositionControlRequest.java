package com.team6962.lib.swerve.config.control;

/**
 * Configuration for a position control request. This does not specify what
 * motion profile will be used or the output type; those are specified in
 * ControlMode. To generate a TalonFX ControlRequest, use
 * {@link ControlMode#createControlRequest()}.
 */
public class PositionControlRequest {
    /**
     * Target position in mechanism units.
     */
    public double Position = 0.0;

    /**
     * The slot to use for closed-loop control and feedforward constants.
     */
    public int Slot = 0;

    /**
     * Whether to use timesync for the control request.
     */
    public boolean UseTimesync = false;

    /**
     * The expected update frequency, in Hz.
     */
    public double UpdateFreqHz = 100;

    /**
     * Create a PositionControlRequest with the given target position.
     * 
     * @param position Target position
     */
    public PositionControlRequest(double position) {
        Position = position;
    }

    /**
     * Create a PositionControlRequest with default values.
     */
    public PositionControlRequest() {
    }

    /**
     * Set the target position in mechanism units.
     * 
     * @param position Target position
     * @return This PositionControlRequest, for chaining
     */
    public PositionControlRequest withPosition(double position) {
        Position = position;
        return this;
    }

    /**
     * Set the slot to use for closed-loop control and feedforward constants.
     * 
     * @param slot The slot index
     * @return This PositionControlRequest, for chaining
     */
    public PositionControlRequest withSlot(int slot) {
        Slot = slot;
        return this;
    }

    /**
     * Set whether to use timesync for the control request.
     * 
     * @param useTimesync Whether to use timesync
     * @return This PositionControlRequest, for chaining
     */
    public PositionControlRequest withUseTimesync(boolean useTimesync) {
        UseTimesync = useTimesync;
        return this;
    }

    /**
     * Set the expected update frequency, in Hz.
     * 
     * @param freqHz The update frequency
     * @return This PositionControlRequest, for chaining
     */
    public PositionControlRequest withUpdateFreqHz(double freqHz) {
        UpdateFreqHz = freqHz;
        return this;
    }
}
