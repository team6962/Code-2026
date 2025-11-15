package com.team6962.lib.swerve.config.control;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

/**
 * Generates control requests to control motor position or velocity with given
 * motion profile and output types. Pass a {@link PositionControlRequest} or
 * {@link VelocityControlRequest} into {@link #createControlRequest()} to
 * generate a TalonFX ControlRequest.
 */
public class ControlMode {
    /**
     * The motion profile type to use.
     */
    public MotionProfileValue MotionProfile = MotionProfileValue.Trapezoidal;

    /**
     * The output type to use.
     */
    public OutputTypeValue OutputType = OutputTypeValue.Voltage;

    /**
     * The motion profile type to use for motor control. The recommended modes
     * for maximum performance are trapezoidal profiling for velocity requests
     * and exponential profiling for position requests.
     */
    public static enum MotionProfileValue {
        /**
         * PID and feedforward control without motion profiling.
         * <p>
         * Uses P, I, D, and S, along with V when creating velocity requests.
         * <p>
         * Can generate either position or velocity requests.
         */
        None,

        /**
         * Trapezoidal motion profiling, or S-curve motion profiling if Motion
         * Magic jerk is configured.
         * <p>
         * Uses P, I, D, S, V, and A, as well as the configured Motion Magic
         * cruise velocity, acceleration, and jerk.
         * <p>
         * Can generate either position or velocity requests.
         */
        Trapezoidal,

        /**
         * Exponential motion profiling.
         * <p>
         * Uses P, I, D, S, V, and A, as well as the configured Motion Magic
         * Expo V and A.
         * <p>
         * Can only generate position requests.
         */
        Exponential
    }

    /**
     * The output type to use for motor control. Field-oriented control (FOC) is
     * recommended, as it improves motor efficiency and power output. There is
     * no significant performance difference between VoltageFOC and
     * TorqueCurrentFOC modes, though their control methods and constants
     * differ.
     */
    public static enum OutputTypeValue {
        /**
         * Uses voltage to indirectly control the torque and velocity of the
         * motor. Uses trapezoidal commutation for motor control.
         */
        Voltage,

        /**
         * Uses voltage to indirectly control the torque and velocity of the
         * motor. Also uses FOC to optimize motor performance.
         */
        VoltageFOC,

        /**
         * Uses torque current with FOC to directly control the torque of the
         * motor.
         */
        TorqueCurrentFOC
    }

    /**
     * Create a ControlMode with the given motion profile and output type.
     * 
     * @param profile  The motion profile
     * @param output  The output type
     */
    public ControlMode(MotionProfileValue profile, OutputTypeValue output) {
        MotionProfile = profile;
        OutputType = output;
    }

    /**
     * Create a ControlMode with default values.
     */
    public ControlMode() {
    }

    /**
     * Set the motion profile to use and return this ControlMode for chaining.
     * 
     * @param profile The motion profile
     * @return        This ControlMode, for chaining
     */
    public ControlMode withMotionProfile(MotionProfileValue profile) {
        MotionProfile = profile;
        return this;
    }

    /**
     * Set the output type to use and return this ControlMode for chaining.
     * 
     * @param output The output type
     * @return       This ControlMode, for chaining
     */
    public ControlMode withOutput(OutputTypeValue output) {
        OutputType = output;
        return this;
    }

    /**
     * Create a TalonFX ControlRequest based on the given
     * PositionControlRequest, using the motion profile and output type
     * configured in this ControlMode.
     * 
     * @param request The PositionControlRequest
     * @return The generated ControlRequest
     */
    public ControlRequest toTalonRequest(PositionControlRequest request) {
        if (MotionProfile == MotionProfileValue.None && (OutputType == OutputTypeValue.Voltage || OutputType == OutputTypeValue.VoltageFOC)) {
            return new PositionVoltage(request.Position)
                .withSlot(request.Slot)
                .withUseTimesync(request.UseTimesync)
                .withUpdateFreqHz(request.UpdateFreqHz)
                .withEnableFOC(OutputType == OutputTypeValue.VoltageFOC);
        } else if (MotionProfile == MotionProfileValue.Trapezoidal && (OutputType == OutputTypeValue.Voltage || OutputType == OutputTypeValue.VoltageFOC)) {
            return new MotionMagicVoltage(request.Position)
                .withSlot(request.Slot)
                .withUseTimesync(request.UseTimesync)
                .withUpdateFreqHz(request.UpdateFreqHz)
                .withEnableFOC(OutputType == OutputTypeValue.VoltageFOC);
        } else if (MotionProfile == MotionProfileValue.Exponential && (OutputType == OutputTypeValue.Voltage || OutputType == OutputTypeValue.VoltageFOC)) {
            return new MotionMagicExpoVoltage(request.Position)
                .withSlot(request.Slot)
                .withUseTimesync(request.UseTimesync)
                .withUpdateFreqHz(request.UpdateFreqHz)
                .withEnableFOC(OutputType == OutputTypeValue.VoltageFOC);
        } else if (MotionProfile == MotionProfileValue.None && OutputType == OutputTypeValue.TorqueCurrentFOC) {
            return new PositionTorqueCurrentFOC(request.Position)
                .withSlot(request.Slot)
                .withUseTimesync(request.UseTimesync)
                .withUpdateFreqHz(request.UpdateFreqHz);
        } else if (MotionProfile == MotionProfileValue.Trapezoidal && OutputType == OutputTypeValue.TorqueCurrentFOC) {
            return new MotionMagicTorqueCurrentFOC(request.Position)
                .withSlot(request.Slot)
                .withUseTimesync(request.UseTimesync)
                .withUpdateFreqHz(request.UpdateFreqHz);
        } else if (MotionProfile == MotionProfileValue.Exponential && OutputType == OutputTypeValue.TorqueCurrentFOC) {
            return new MotionMagicExpoTorqueCurrentFOC(request.Position)
                .withSlot(request.Slot)
                .withUseTimesync(request.UseTimesync)
                .withUpdateFreqHz(request.UpdateFreqHz);
        } else {
            return new NeutralOut();
        }
    }

    /**
     * Create a TalonFX ControlRequest based on the given
     * VelocityControlRequest, using the motion profile and output type
     * configured in this ControlMode.
     * 
     * @param request The PositionControlRequest
     * @return The generated ControlRequest
     */
    public ControlRequest toTalonRequest(VelocityControlRequest request) {
        if (MotionProfile == MotionProfileValue.None && (OutputType == OutputTypeValue.Voltage || OutputType == OutputTypeValue.VoltageFOC)) {
            return new VelocityVoltage(request.Velocity)
                .withSlot(request.Slot)
                .withUseTimesync(request.UseTimesync)
                .withUpdateFreqHz(request.UpdateFreqHz)
                .withEnableFOC(OutputType == OutputTypeValue.VoltageFOC);
        } else if (MotionProfile == MotionProfileValue.Trapezoidal && (OutputType == OutputTypeValue.Voltage || OutputType == OutputTypeValue.VoltageFOC)) {
            return new MotionMagicVelocityVoltage(request.Velocity)
                .withSlot(request.Slot)
                .withUseTimesync(request.UseTimesync)
                .withUpdateFreqHz(request.UpdateFreqHz)
                .withEnableFOC(OutputType == OutputTypeValue.VoltageFOC);
        } else if (MotionProfile == MotionProfileValue.None && OutputType == OutputTypeValue.TorqueCurrentFOC) {
            return new VelocityTorqueCurrentFOC(request.Velocity)
                .withSlot(request.Slot)
                .withUseTimesync(request.UseTimesync)
                .withUpdateFreqHz(request.UpdateFreqHz);
        } else if (MotionProfile == MotionProfileValue.Trapezoidal && OutputType == OutputTypeValue.TorqueCurrentFOC) {
            return new MotionMagicVelocityTorqueCurrentFOC(request.Velocity)
                .withSlot(request.Slot)
                .withUseTimesync(request.UseTimesync)
                .withUpdateFreqHz(request.UpdateFreqHz);
        } else {
            return new NeutralOut();
        }
    }
}
