package com.team6962.lib.phoenix.control;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

class DynamicPositionControlRequestTest {

    @Test
    void constructorWithDouble() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0);
        assertEquals(5.0, request.Position);
    }

    @Test
    void constructorWithAngle() {
        Angle position = Rotations.of(3.5);
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(position);
        assertEquals(3.5, request.Position, 0.0001);
    }

    @Test
    void constructorWithConstraints() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0, 10.0, 20.0, 30.0);
        assertEquals(5.0, request.Position);
        assertEquals(10.0, request.Velocity);
        assertEquals(20.0, request.Acceleration);
        assertEquals(30.0, request.Jerk);
    }

    @Test
    void defaultValues() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0);
        assertEquals(0, request.Slot);
        assertEquals(50.0, request.UpdateFreqHz);
        assertFalse(request.UseTimesync);
        assertEquals(DynamicPositionMotionProfileType.Exponential, request.MotionProfileType);
        assertEquals(ControlOutputType.VoltageFOC, request.OutputType);
        assertEquals(0.0, request.Velocity);
        assertEquals(0.0, request.Acceleration);
        assertEquals(0.0, request.Jerk);
        assertEquals(0.0, request.KV);
        assertEquals(0.0, request.KA);
    }

    @Test
    void withPositionDouble() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withPosition(10.0);
        assertEquals(10.0, request.Position);
    }

    @Test
    void withPositionAngle() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withPosition(Rotations.of(7.5));
        assertEquals(7.5, request.Position, 0.0001);
    }

    @Test
    void withVelocityDouble() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withVelocity(15.0);
        assertEquals(15.0, request.Velocity);
    }

    @Test
    void withVelocityAngularVelocity() {
        AngularVelocity velocity = RotationsPerSecond.of(12.5);
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withVelocity(velocity);
        assertEquals(12.5, request.Velocity, 0.0001);
    }

    @Test
    void withAccelerationDouble() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withAcceleration(25.0);
        assertEquals(25.0, request.Acceleration);
    }

    @Test
    void withAccelerationAngularAcceleration() {
        AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(18.5);
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withAcceleration(acceleration);
        assertEquals(18.5, request.Acceleration, 0.0001);
    }

    @Test
    void withJerkDouble() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withJerk(35.0);
        assertEquals(35.0, request.Jerk);
    }

    @Test
    void withSlot() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withSlot(2);
        assertEquals(2, request.Slot);
    }

    @Test
    void withUpdateFreqHz() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withUpdateFreqHz(100.0);
        assertEquals(100.0, request.UpdateFreqHz);
    }

    @Test
    void withUseTimesync() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withUseTimesync(true);
        assertTrue(request.UseTimesync);
    }

    @Test
    void withMotionProfileType() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withMotionProfileType(DynamicPositionMotionProfileType.Trapezoidal);
        assertEquals(DynamicPositionMotionProfileType.Trapezoidal, request.MotionProfileType);
    }

    @Test
    void withOutputType() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);
        assertEquals(ControlOutputType.TorqueCurrentFOC, request.OutputType);
    }

    @Test
    void chainingMethods() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(1.0)
            .withPosition(2.0)
            .withVelocity(10.0)
            .withAcceleration(20.0)
            .withJerk(30.0)
            .withSlot(1)
            .withUpdateFreqHz(200.0)
            .withUseTimesync(true)
            .withMotionProfileType(DynamicPositionMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.Voltage);

        assertEquals(2.0, request.Position);
        assertEquals(10.0, request.Velocity);
        assertEquals(20.0, request.Acceleration);
        assertEquals(30.0, request.Jerk);
        assertEquals(1, request.Slot);
        assertEquals(200.0, request.UpdateFreqHz);
        assertTrue(request.UseTimesync);
        assertEquals(DynamicPositionMotionProfileType.Trapezoidal, request.MotionProfileType);
        assertEquals(ControlOutputType.Voltage, request.OutputType);
    }

    @Test
    void toControlRequest_TrapezoidalVoltage() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0)
            .withVelocity(10.0)
            .withAcceleration(20.0)
            .withMotionProfileType(DynamicPositionMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.Voltage);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(DynamicMotionMagicVoltage.class, result);
    }

    @Test
    void toControlRequest_TrapezoidalVoltageFOC() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0)
            .withVelocity(10.0)
            .withAcceleration(20.0)
            .withMotionProfileType(DynamicPositionMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.VoltageFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(DynamicMotionMagicVoltage.class, result);
    }

    @Test
    void toControlRequest_TrapezoidalTorqueCurrentFOC() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0)
            .withVelocity(10.0)
            .withAcceleration(20.0)
            .withMotionProfileType(DynamicPositionMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(DynamicMotionMagicTorqueCurrentFOC.class, result);
    }

    @Test
    void toControlRequest_ExponentialVoltage() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0)
            .withMotionProfileType(DynamicPositionMotionProfileType.Exponential)
            .withOutputType(ControlOutputType.Voltage);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(DynamicMotionMagicExpoVoltage.class, result);
    }

    @Test
    void toControlRequest_ExponentialVoltageFOC() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0)
            .withMotionProfileType(DynamicPositionMotionProfileType.Exponential)
            .withOutputType(ControlOutputType.VoltageFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(DynamicMotionMagicExpoVoltage.class, result);
    }

    @Test
    void toControlRequest_ExponentialTorqueCurrentFOC() {
        DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0)
            .withMotionProfileType(DynamicPositionMotionProfileType.Exponential)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(DynamicMotionMagicExpoTorqueCurrentFOC.class, result);
    }
}
