package com.team6962.lib.swerve.config;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

public final class ControlMode {
    private ControlMode() {
    }

    public static enum VelocityMotionProfileValue {
        None,
        Trapezoidal
    }

    public static enum PositionMotionProfileValue {
        None,
        Trapezoidal,
        Exponential
    }

    public static enum OutputTypeValue {
        Voltage,
        VoltageFOC,
        TorqueCurrentFOC
    }

    public static class Position {
        public PositionMotionProfileValue MotionProfile = PositionMotionProfileValue.None;
        public OutputTypeValue OutputType = OutputTypeValue.VoltageFOC;

        public Position(PositionMotionProfileValue motionProfile, OutputTypeValue outputType) {
            MotionProfile = motionProfile;
            OutputType = outputType;
        }

        public Position() {
        }

        public ControlRequest createRequest(double position, int slot, double updateFreqHz, boolean useTimesync) {
            switch (MotionProfile) {
                case None:
                    switch (OutputType) {
                        case Voltage:
                            return new PositionVoltage(position)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync)
                                .withEnableFOC(false);
                        case VoltageFOC:
                            return new PositionVoltage(position)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync)
                                .withEnableFOC(true);
                        case TorqueCurrentFOC:
                            return new PositionTorqueCurrentFOC(position)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync);
                    }
                case Trapezoidal:
                    switch (OutputType) {
                        case Voltage:
                            return new MotionMagicVelocityVoltage(position)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync)
                                .withEnableFOC(false);
                        case VoltageFOC:
                            return new MotionMagicVelocityVoltage(position)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync)
                                .withEnableFOC(true);
                        case TorqueCurrentFOC:
                            return new MotionMagicVelocityTorqueCurrentFOC(position)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync);
                    }
                case Exponential:
                    switch (OutputType) {
                        case Voltage:
                            return new MotionMagicExpoVoltage(position)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync)
                                .withEnableFOC(false);
                        case VoltageFOC:
                            return new MotionMagicExpoVoltage(position)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync)
                                .withEnableFOC(true);
                        case TorqueCurrentFOC:
                            return new MotionMagicExpoTorqueCurrentFOC(position)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync);
                    }
            }

            return new NeutralOut();
        }
    }

    public static class Velocity {
        public VelocityMotionProfileValue MotionProfile = VelocityMotionProfileValue.None;
        public OutputTypeValue OutputType = OutputTypeValue.VoltageFOC;

        public Velocity(VelocityMotionProfileValue motionProfile, OutputTypeValue outputType) {
            MotionProfile = motionProfile;
            OutputType = outputType;
        }

        public Velocity() {
        }

        public ControlRequest createRequest(double velocity, int slot, double updateFreqHz, boolean useTimesync) {
            switch (MotionProfile) {
                case None:
                    switch (OutputType) {
                        case Voltage:
                            return new VelocityVoltage(velocity)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync)
                                .withEnableFOC(false);
                        case VoltageFOC:
                            return new VelocityVoltage(velocity)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync)
                                .withEnableFOC(true);
                        case TorqueCurrentFOC:
                            return new VelocityTorqueCurrentFOC(velocity)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync);
                    }
                case Trapezoidal:
                    switch (OutputType) {
                        case Voltage:
                            return new MotionMagicVelocityVoltage(velocity)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync)
                                .withEnableFOC(false);
                        case VoltageFOC:
                            return new MotionMagicVelocityVoltage(velocity)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync)
                                .withEnableFOC(true);
                        case TorqueCurrentFOC:
                            return new MotionMagicVelocityTorqueCurrentFOC(velocity)
                                .withSlot(slot)
                                .withUpdateFreqHz(updateFreqHz)
                                .withUseTimesync(useTimesync);
                    }
            }

            return new NeutralOut();
        }
    }

    public static class DynamicallyConstrainedPosition {
        public OutputTypeValue OutputType = OutputTypeValue.VoltageFOC;

        public ControlRequest createRequest(double position, double velocity, double acceleration, double jerk, int slot, double updateFreqHz, boolean useTimesync) {
            switch (OutputType) {
                case Voltage:
                    return new DynamicMotionMagicVoltage(position, velocity, acceleration, jerk)
                        .withSlot(slot)
                        .withUpdateFreqHz(updateFreqHz)
                        .withUseTimesync(useTimesync)
                        .withEnableFOC(false);
                case VoltageFOC:
                    return new DynamicMotionMagicVoltage(position, velocity, acceleration, jerk)
                        .withSlot(slot)
                        .withUpdateFreqHz(updateFreqHz)
                        .withUseTimesync(useTimesync)
                        .withEnableFOC(true);
                case TorqueCurrentFOC:
                    return new DynamicMotionMagicTorqueCurrentFOC(position, velocity, acceleration, jerk)
                        .withSlot(slot)
                        .withUpdateFreqHz(updateFreqHz)
                        .withUseTimesync(useTimesync);
            }

            return new NeutralOut();
        }
    }
}
