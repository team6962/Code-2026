package com.team6962.lib.phoenix.control;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Configuration for a control request that can be used to control the position
 * of a motor with dynamic constraints. While the Phoenix 6 library has control
 * requests for each possible combination of motion profile types and output
 * types, this class combines them into a single object with enums to select the
 * desired types. This allows dynamic position control requests of different
 * classes to be created based on two configuration constants.
 * <p>
 * A {@link DynamicPositionControlRequest} cannot be directly applied to a motor
 * controller; it must first be converted to a {@link ControlRequest} object
 * using the {@link #toControlRequest()} method.
 */
public class DynamicPositionControlRequest {
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
    public DynamicPositionMotionProfileType MotionProfileType = DynamicPositionMotionProfileType.Exponential;

    /**
     * The output type to use for this position control request.
     */
    public ControlOutputType OutputType = ControlOutputType.VoltageFOC;

    /**
     * The maximum velocity constraint in rotations per second.
     */
    public double Velocity = 0.0;

    /**
     * The maximum acceleration constraint in rotations per second squared.
     */
    public double Acceleration = 0.0;

    /**
     * The maximum jerk constraint in rotations per second cubed.
     */
    public double Jerk = 0.0;

    /**
     * The KV exponential constraint in (rotations per second) / volt.
     */
    public double KV = 0.0;

    /**
     * The KA exponential constraint in (rotations per second squared) / volt.
     */
    public double KA = 0.0;

    /**
     * Constructs a control request with the given target position.
     * 
     * @param Position The position to drive toward in rotations.
     */
    public DynamicPositionControlRequest(double Position) {
        this.Position = Position;
    }

    /**
     * Constructs a control request with the given target position.
     * 
     * @param Position The position to drive toward as an Angle.
     */
    public DynamicPositionControlRequest(Angle Position) {
        this.Position = Position.in(Rotations);
    }

    /**
     * Constructs a control request with the given target position and
     * trapezoidal constraints.
     * 
     * @param Position The position to drive toward in rotations.
     * @param Velocity The maximum velocity constraint in rotations per
     *                 second.
     * @param Acceleration The maximum acceleration constraint in rotations per
     *                     second squared.
     * @param Jerk The maximum jerk constraint in rotations per second
     *             cubed.
     */
    public DynamicPositionControlRequest(double Position, double Velocity, double Acceleration, double Jerk) {
        this.Position = Position;
        this.Velocity = Velocity;
        this.Acceleration = Acceleration;
        this.Jerk = Jerk;
    }

    /**
     * Sets the maximum velocity constraint for this position control request.
     * 
     * @param velocity The velocity to drive toward in rotations per second.
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withVelocity(double velocity) {
        this.Velocity = velocity;
        return this;
    }

    /**
     * Sets the maximum velocity constraint for this position control request.
     * 
     * @param velocity The velocity to drive toward as an AngularVelocity.
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withVelocity(AngularVelocity velocity) {
        this.Velocity = velocity.in(RotationsPerSecond);
        return this;
    }

    /**
     * Sets the maximum acceleration constraint for this position control request.
     * 
     * @param acceleration The acceleration to drive toward in rotations per second squared.
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withAcceleration(double acceleration) {
        this.Acceleration = acceleration;
        return this;
    }

    /**
     * Sets the maximum acceleration constraint for this position control request.
     * 
     * @param acceleration The acceleration to drive toward as an AngularAcceleration.
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withAcceleration(AngularAcceleration acceleration) {
        this.Acceleration = acceleration.in(RotationsPerSecondPerSecond);
        return this;
    }

    /**
     * Sets the maximum jerk constraint for this position control request.
     * 
     * @param jerk The jerk to drive toward in rotations per second cubed.
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withJerk(double jerk) {
        this.Jerk = jerk;
        return this;
    }

    /**
     * Sets the maximum jerk constraint for this position control request.
     * 
     * @param jerk The jerk to drive toward as an AngularAcceleration per second.
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withJerk(Measure<PerUnit<AngularAccelerationUnit, TimeUnit>> jerk) {
        this.Jerk = jerk.in(RotationsPerSecondPerSecond.per(Second));
        return this;
    }

    /**
     * Sets the target position for this position control request.
     * 
     * @param position The position to drive toward in rotations.
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withPosition(double position) {
        this.Position = position;
        return this;
    }

    /**
     * Sets the target position for this position control request.
     * 
     * @param position The position to drive toward as an Angle.
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withPosition(Angle position) {
        this.Position = position.in(Rotations);
        return this;
    }

    /**
     * Sets the slot index for this position control request.
     * 
     * @param slot The slot index (from 0 to 2)
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withSlot(int slot) {
        this.Slot = slot;
        return this;
    }

    /**
     * Sets the update frequency for this position control request.
     * 
     * @param updateFreqHz The update frequency in Hertz
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withUpdateFreqHz(double updateFreqHz) {
        this.UpdateFreqHz = updateFreqHz;
        return this;
    }

    /**
     * Sets whether to use timesync for this position control request.
     * 
     * @param useTimesync True to use timesync
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withUseTimesync(boolean useTimesync) {
        this.UseTimesync = useTimesync;
        return this;
    }

    /**
     * Sets the motion profile type for this position control request.
     * 
     * @param motionProfileType The motion profile type
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withMotionProfileType(DynamicPositionMotionProfileType motionProfileType) {
        this.MotionProfileType = motionProfileType;
        return this;
    }

    /**
     * Sets the output type for this position control request.
     * 
     * @param outputType The output type
     * @return This control request object, for chaining.
     */
    public DynamicPositionControlRequest withOutputType(ControlOutputType outputType) {
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
            case Exponential:
                switch (OutputType) {
                    case Voltage:
                        return new DynamicMotionMagicExpoVoltage(Position, KV, KA)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(false);
                    case VoltageFOC:
                        return new DynamicMotionMagicExpoVoltage(Position, KV, KA)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(true);
                    case TorqueCurrentFOC:
                        return new DynamicMotionMagicExpoTorqueCurrentFOC(Position, KV, KA)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync);
                }
            case Trapezoidal:
                switch (OutputType) {
                    case Voltage:
                        return new DynamicMotionMagicVoltage(Position, Velocity, Acceleration)
                            .withJerk(Jerk)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(false);
                    case VoltageFOC:
                        return new DynamicMotionMagicVoltage(Position, Velocity, Acceleration)
                            .withJerk(Jerk)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync)
                            .withEnableFOC(true);
                    case TorqueCurrentFOC:
                        return new DynamicMotionMagicTorqueCurrentFOC(Position, Velocity, Acceleration)
                            .withJerk(Jerk)
                            .withSlot(Slot)
                            .withUpdateFreqHz(UpdateFreqHz)
                            .withUseTimesync(UseTimesync);
                }
        }

        throw new IllegalArgumentException("Unsupported MotionProfileType (" +
            MotionProfileType + ") or OutputType (" + OutputType + ")");
    }
}
