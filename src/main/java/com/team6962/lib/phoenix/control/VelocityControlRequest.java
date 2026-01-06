package com.team6962.lib.phoenix.control;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Configuration for a control request that can be used to control the velocity
 * of a motor. While the Phoenix 6 library has control requests for each
 * possible combination of motion profile types and output types, this class
 * combines them into a single object with enums to select the desired types.
 * This allows velocity control requests of different classes to be created
 * based on two configuration constants.
 * <p>
 * A {@link VelocityControlRequest} cannot be directly applied to a motor
 * controller; it must first be converted to a {@link ControlRequest} object
 * using the {@link #toControlRequest()} method.
 */
public class VelocityControlRequest {
    /**
     * Velocity to drive toward in rotations per second.
     */
    public double Velocity = 0.0;

    /**
     * The slot index (from 0 to 2) of the slot where PID and feedforward
     * constants are stored in the motor controller configuration.
     */
    public int Slot = 0;

    /**
     * The period at which this control request is updated at, in Hertz.
     */
    public double UpdateFreqHz = 50.0;

    /**
     * Set to true to delay applying this control request until a timesync boundary
     * (requires Phoenix Pro and CANivore). This eliminates the impact of
     * nondeterministic network delays in exchange for a larger but deterministic
     * control latency.
     */
    public boolean UseTimesync = false;

    /**
     * The motion profile type to use for this velocity control request.
     */
    public VelocityMotionProfileType MotionProfileType = VelocityMotionProfileType.Trapezoidal;

    /**
     * The output type to use for this velocity control request.
     */
    public ControlOutputType OutputType = ControlOutputType.VoltageFOC;

    /**
     * Constructs a control request with the given target velocity.
     * 
     * @param Velocity The velocity to drive toward in rotations per second.
     */
    public VelocityControlRequest(double Velocity) {
        this.Velocity = Velocity;
    }

    /**
     * Constructs a control request with the given target velocity.
     * 
     * @param Velocity The velocity to drive toward as an AngularVelocity.
     */
    public VelocityControlRequest(AngularVelocity Velocity) {
        this.Velocity = Velocity.in(RotationsPerSecond);
    }

    /**
     * Sets the target velocity for this velocity control request.
     * 
     * @param velocity The velocity to drive toward in rotations per second.
     * @return This control request object, for chaining.
     */
    public VelocityControlRequest withVelocity(double velocity) {
        this.Velocity = velocity;
        return this;
    }

    /**
     * Sets the target velocity for this velocity control request.
     * 
     * @param velocity The velocity to drive toward as an AngularVelocity.
     * @return This control request object, for chaining.
     */
    public VelocityControlRequest withVelocity(AngularVelocity velocity) {
        this.Velocity = velocity.in(RotationsPerSecond);
        return this;
    }

    /**
     * Sets the slot index for this velocity control request.
     * 
     * @param slot The slot index (from 0 to 2)
     * @return This control request object, for chaining.
     */
    public VelocityControlRequest withSlot(int slot) {
        this.Slot = slot;
        return this;
    }

    /**
     * Sets the update frequency for this velocity control request.
     * 
     * @param updateFreqHz The update frequency in Hertz
     * @return This control request object, for chaining.
     */
    public VelocityControlRequest withUpdateFreqHz(double updateFreqHz) {
        this.UpdateFreqHz = updateFreqHz;
        return this;
    }

    /**
     * Sets whether to use timesync for this velocity control request.
     * 
     * @param useTimesync True to use timesync
     * @return This control request object, for chaining.
     */
    public VelocityControlRequest withUseTimesync(boolean useTimesync) {
        this.UseTimesync = useTimesync;
        return this;
    }

    /**
     * Sets the motion profile type for this velocity control request.
     * 
     * @param motionProfileType The motion profile type
     * @return This control request object, for chaining.
     */
    public VelocityControlRequest withMotionProfileType(VelocityMotionProfileType motionProfileType) {
        this.MotionProfileType = motionProfileType;
        return this;
    }

    /**
     * Sets the output type for this velocity control request.
     * 
     * @param outputType The output type
     * @return This control request object, for chaining.
     */
    public VelocityControlRequest withOutputType(ControlOutputType outputType) {
        this.OutputType = outputType;
        return this;
    }

    /**
     * Converts this control request object to a {@link ControlRequest} object.
     */
    public ControlRequest toControlRequest() {
        if (MotionProfileType == null) {
            throw new IllegalStateException("MotionProfileType cannot be null");
        }

        if (OutputType == null) {
            throw new IllegalStateException("OutputType cannot be null");
        }

        switch (MotionProfileType) {
            case None:
                switch (OutputType) {
                    case Voltage:
                        return new VelocityVoltage(Velocity)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(false);
                    case VoltageFOC:
                        return new VelocityVoltage(Velocity)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(true);
                    case TorqueCurrentFOC:
                        return new VelocityTorqueCurrentFOC(Velocity)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync);
                }
            case Trapezoidal:
                switch (OutputType) {
                    case Voltage:
                        return new MotionMagicVelocityVoltage(Velocity)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(false);
                    case VoltageFOC:
                        return new MotionMagicVelocityVoltage(Velocity)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(true);
                    case TorqueCurrentFOC:
                        return new MotionMagicVelocityTorqueCurrentFOC(Velocity)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync);
                }
        }

        throw new IllegalArgumentException("Unsupported MotionProfileType (" +
            MotionProfileType + ") or OutputType (" + OutputType + ")");
    }
}
