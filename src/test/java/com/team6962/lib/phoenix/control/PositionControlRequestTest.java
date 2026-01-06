package com.team6962.lib.phoenix.control;

import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.units.measure.Angle;

class PositionControlRequestTest {

    @Test
    void constructorWithDouble() {
        PositionControlRequest request = new PositionControlRequest(5.0);
        assertEquals(5.0, request.Position);
    }

    @Test
    void constructorWithAngle() {
        Angle position = Rotations.of(3.5);
        PositionControlRequest request = new PositionControlRequest(position);
        assertEquals(3.5, request.Position, 0.0001);
    }

    @Test
    void defaultValues() {
        PositionControlRequest request = new PositionControlRequest(0.0);
        assertEquals(0, request.Slot);
        assertEquals(50.0, request.UpdateFreqHz);
        assertFalse(request.UseTimesync);
        assertEquals(PositionMotionProfileType.Exponential, request.MotionProfileType);
        assertEquals(ControlOutputType.VoltageFOC, request.OutputType);
    }

    @Test
    void withPositionDouble() {
        PositionControlRequest request = new PositionControlRequest(0.0)
            .withPosition(10.0);
        assertEquals(10.0, request.Position);
    }

    @Test
    void withPositionAngle() {
        PositionControlRequest request = new PositionControlRequest(0.0)
            .withPosition(Rotations.of(7.5));
        assertEquals(7.5, request.Position, 0.0001);
    }

    @Test
    void withSlot() {
        PositionControlRequest request = new PositionControlRequest(0.0)
            .withSlot(2);
        assertEquals(2, request.Slot);
    }

    @Test
    void withUpdateFreqHz() {
        PositionControlRequest request = new PositionControlRequest(0.0)
            .withUpdateFreqHz(100.0);
        assertEquals(100.0, request.UpdateFreqHz);
    }

    @Test
    void withUseTimesync() {
        PositionControlRequest request = new PositionControlRequest(0.0)
            .withUseTimesync(true);
        assertTrue(request.UseTimesync);
    }

    @Test
    void withMotionProfileType() {
        PositionControlRequest request = new PositionControlRequest(0.0)
            .withMotionProfileType(PositionMotionProfileType.Trapezoidal);
        assertEquals(PositionMotionProfileType.Trapezoidal, request.MotionProfileType);
    }

    @Test
    void withOutputType() {
        PositionControlRequest request = new PositionControlRequest(0.0)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);
        assertEquals(ControlOutputType.TorqueCurrentFOC, request.OutputType);
    }

    @Test
    void chainingMethods() {
        PositionControlRequest request = new PositionControlRequest(1.0)
            .withPosition(2.0)
            .withSlot(1)
            .withUpdateFreqHz(200.0)
            .withUseTimesync(true)
            .withMotionProfileType(PositionMotionProfileType.None)
            .withOutputType(ControlOutputType.Voltage);

        assertEquals(2.0, request.Position);
        assertEquals(1, request.Slot);
        assertEquals(200.0, request.UpdateFreqHz);
        assertTrue(request.UseTimesync);
        assertEquals(PositionMotionProfileType.None, request.MotionProfileType);
        assertEquals(ControlOutputType.Voltage, request.OutputType);
    }

    @Test
    void toControlRequest_NoneVoltage() {
        PositionControlRequest request = new PositionControlRequest(5.0)
            .withMotionProfileType(PositionMotionProfileType.None)
            .withOutputType(ControlOutputType.Voltage);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(PositionVoltage.class, result);
    }

    @Test
    void toControlRequest_NoneVoltageFOC() {
        PositionControlRequest request = new PositionControlRequest(5.0)
            .withMotionProfileType(PositionMotionProfileType.None)
            .withOutputType(ControlOutputType.VoltageFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(PositionVoltage.class, result);
    }

    @Test
    void toControlRequest_NoneTorqueCurrentFOC() {
        PositionControlRequest request = new PositionControlRequest(5.0)
            .withMotionProfileType(PositionMotionProfileType.None)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(PositionTorqueCurrentFOC.class, result);
    }

    @Test
    void toControlRequest_TrapezoidalVoltage() {
        PositionControlRequest request = new PositionControlRequest(5.0)
            .withMotionProfileType(PositionMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.Voltage);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(MotionMagicVelocityVoltage.class, result);
    }

    @Test
    void toControlRequest_TrapezoidalVoltageFOC() {
        PositionControlRequest request = new PositionControlRequest(5.0)
            .withMotionProfileType(PositionMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.VoltageFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(MotionMagicVelocityVoltage.class, result);
    }

    @Test
    void toControlRequest_TrapezoidalTorqueCurrentFOC() {
        PositionControlRequest request = new PositionControlRequest(5.0)
            .withMotionProfileType(PositionMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(MotionMagicVelocityTorqueCurrentFOC.class, result);
    }

    @Test
    void toControlRequest_ExponentialVoltage() {
        PositionControlRequest request = new PositionControlRequest(5.0)
            .withMotionProfileType(PositionMotionProfileType.Exponential)
            .withOutputType(ControlOutputType.Voltage);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(MotionMagicExpoVoltage.class, result);
    }

    @Test
    void toControlRequest_ExponentialVoltageFOC() {
        PositionControlRequest request = new PositionControlRequest(5.0)
            .withMotionProfileType(PositionMotionProfileType.Exponential)
            .withOutputType(ControlOutputType.VoltageFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(MotionMagicExpoVoltage.class, result);
    }

    @Test
    void toControlRequest_ExponentialTorqueCurrentFOC() {
        PositionControlRequest request = new PositionControlRequest(5.0)
            .withMotionProfileType(PositionMotionProfileType.Exponential)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(MotionMagicExpoTorqueCurrentFOC.class, result);
    }
}
