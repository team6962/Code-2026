package com.team6962.lib.swerve.config.control;

/**
 * Configuration for a velocity control request. This does not specify what
 * motion profile will be used or the output type; those are specified in
 * ControlMode. To generate a TalonFX ControlRequest, use
 * {@link ControlMode#createControlRequest()}.
 */
public class VelocityControlRequest {
    /**
     * Target velocity in mechanism units.
     */
    public double Velocity = 0.0;

    /**
     * The slot to use for closed-loop control and feedforward constants.
     */
    public int Slot = 1;

    /**
     * Whether to use timesync for the control request.
     */
    public boolean UseTimesync = false;

    /**
     * The expected update frequency, in Hz.
     */
    public double UpdateFreqHz = 100;

    /**
     * Create a VelocityControlRequest with the given target velocity.
     * 
     * @param velocity Target velocity
     */
    public VelocityControlRequest(double velocity) {
        Velocity = velocity;
    }

    /**
     * Create a VelocityControlRequest with default values.
     */
    public VelocityControlRequest() {
    }

    /**
     * Set the target velocity in mechanism units.
     * 
     * @param velocity Target velocity
     * @return This VelocityControlRequest, for chaining
     */
    public VelocityControlRequest withPosition(double velocity) {
        Velocity = velocity;
        return this;
    }

    /**
     * Set the slot to use for closed-loop control and feedforward constants.
     * 
     * @param slot The slot index
     * @return This VelocityControlRequest, for chaining
     */
    public VelocityControlRequest withSlot(int slot) {
        Slot = slot;
        return this;
    }

    /**
     * Set whether to use timesync for the control request.
     * 
     * @param useTimesync Whether to use timesync
     * @return This VelocityControlRequest, for chaining
     */
    public VelocityControlRequest withUseTimesync(boolean useTimesync) {
        UseTimesync = useTimesync;
        return this;
    }

    /**
     * Set the expected update frequency, in Hz.
     * 
     * @param freqHz The update frequency
     * @return This VelocityControlRequest, for chaining
     */
    public VelocityControlRequest withUpdateFreqHz(double freqHz) {
        UpdateFreqHz = freqHz;
        return this;
    }
}
