package com.team6962.lib.phoenix.control;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.units.measure.Angle;

/**
 * Configuration for a control request that can be used to control the position
 * of a motor. While the Phoenix 6 library has control requests for each
 * possible combination of motion profile types and output types, this class
 * combines them into a single object with enums to select the desired types.
 * This allows position control requests of different classes to be created
 * based on two configuration constants.
 * <p>
 * A {@link PositionControlRequest} cannot be directly applied to a motor
 * controller; it must first be converted to a {@link ControlRequest} object
 * using the {@link #toControlRequest()} method.
 */
public class PositionControlRequest {
    /**
     * Position to drive toward in rotations.
     */
    public double Position = 0.0;

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
     * The motion profile type to use for this position control request.
     */
    public PositionMotionProfileType MotionProfileType = PositionMotionProfileType.Exponential;

    /**
     * The output type to use for this position control request.
     */
    public ControlOutputType OutputType = ControlOutputType.VoltageFOC;

    /**
     * Constructs a control request with the given target position.
     * 
     * @param Position The position to drive toward in rotations.
     */
    public PositionControlRequest(double Position) {
        this.Position = Position;
    }

    /**
     * Constructs a control request with the given target position.
     * 
     * @param Position The position to drive toward as an Angle.
     */
    public PositionControlRequest(Angle Position) {
        this.Position = Position.in(Rotations);
    }

    /**
     * Sets the target position for this position control request.
     * 
     * @param position The position to drive toward in rotations.
     * @return This control request object, for chaining.
     */
    public PositionControlRequest withPosition(double position) {
        this.Position = position;
        return this;
    }

    /**
     * Sets the target position for this position control request.
     * 
     * @param position The position to drive toward as an Angle.
     * @return This control request object, for chaining.
     */
    public PositionControlRequest withPosition(Angle position) {
        this.Position = position.in(Rotations);
        return this;
    }

    /**
     * Sets the slot index for this position control request.
     * 
     * @param slot The slot index (from 0 to 2)
     * @return This control request object, for chaining.
     */
    public PositionControlRequest withSlot(int slot) {
        this.Slot = slot;
        return this;
    }

    /**
     * Sets the update frequency for this position control request.
     * 
     * @param updateFreqHz The update frequency in Hertz
     * @return This control request object, for chaining.
     */
    public PositionControlRequest withUpdateFreqHz(double updateFreqHz) {
        this.UpdateFreqHz = updateFreqHz;
        return this;
    }

    /**
     * Sets whether to use timesync for this position control request.
     * 
     * @param useTimesync True to use timesync
     * @return This control request object, for chaining.
     */
    public PositionControlRequest withUseTimesync(boolean useTimesync) {
        this.UseTimesync = useTimesync;
        return this;
    }

    /**
     * Sets the motion profile type for this position control request.
     * 
     * @param motionProfileType The motion profile type
     * @return This control request object, for chaining.
     */
    public PositionControlRequest withMotionProfileType(PositionMotionProfileType motionProfileType) {
        this.MotionProfileType = motionProfileType;
        return this;
    }

    /**
     * Sets the output type for this position control request.
     * 
     * @param outputType The output type
     * @return This control request object, for chaining.
     */
    public PositionControlRequest withOutputType(ControlOutputType outputType) {
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
                        return new PositionVoltage(Position)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(false);
                    case VoltageFOC:
                        return new PositionVoltage(Position)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(true);
                    case TorqueCurrentFOC:
                        return new PositionTorqueCurrentFOC(Position)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync);
                }
            case Trapezoidal:
                switch (OutputType) {
                    case Voltage:
                        return new MotionMagicVelocityVoltage(Position)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(false);
                    case VoltageFOC:
                        return new MotionMagicVelocityVoltage(Position)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(true);
                    case TorqueCurrentFOC:
                        return new MotionMagicVelocityTorqueCurrentFOC(Position)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync);
                }
            case Exponential:
                switch (OutputType) {
                    case Voltage:
                        return new MotionMagicExpoVoltage(Position)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(false);
                    case VoltageFOC:
                        return new MotionMagicExpoVoltage(Position)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(true);
                    case TorqueCurrentFOC:
                        return new MotionMagicExpoTorqueCurrentFOC(Position)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync);
                }
        }

        throw new IllegalArgumentException("Unsupported MotionProfileType (" +
            MotionProfileType + ") or OutputType (" + OutputType + ")");
    }
}
