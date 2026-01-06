package com.team6962.lib.phoenix.control;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.units.measure.AngularVelocity;

class VelocityControlRequestTest {

    @Test
    void constructorWithDouble() {
        VelocityControlRequest request = new VelocityControlRequest(5.0);
        assertEquals(5.0, request.Velocity);
    }

    @Test
    void constructorWithAngularVelocity() {
        AngularVelocity velocity = RotationsPerSecond.of(3.5);
        VelocityControlRequest request = new VelocityControlRequest(velocity);
        assertEquals(3.5, request.Velocity, 0.0001);
    }

    @Test
    void defaultValues() {
        VelocityControlRequest request = new VelocityControlRequest(0.0);
        assertEquals(0, request.Slot);
        assertEquals(50.0, request.UpdateFreqHz);
        assertFalse(request.UseTimesync);
        assertEquals(VelocityMotionProfileType.Trapezoidal, request.MotionProfileType);
        assertEquals(ControlOutputType.VoltageFOC, request.OutputType);
    }

    @Test
    void withVelocityDouble() {
        VelocityControlRequest request = new VelocityControlRequest(0.0)
            .withVelocity(10.0);
        assertEquals(10.0, request.Velocity);
    }

    @Test
    void withVelocityAngularVelocity() {
        VelocityControlRequest request = new VelocityControlRequest(0.0)
            .withVelocity(RotationsPerSecond.of(7.5));
        assertEquals(7.5, request.Velocity, 0.0001);
    }

    @Test
    void withSlot() {
        VelocityControlRequest request = new VelocityControlRequest(0.0)
            .withSlot(2);
        assertEquals(2, request.Slot);
    }

    @Test
    void withUpdateFreqHz() {
        VelocityControlRequest request = new VelocityControlRequest(0.0)
            .withUpdateFreqHz(100.0);
        assertEquals(100.0, request.UpdateFreqHz);
    }

    @Test
    void withUseTimesync() {
        VelocityControlRequest request = new VelocityControlRequest(0.0)
            .withUseTimesync(true);
        assertTrue(request.UseTimesync);
    }

    @Test
    void withMotionProfileType() {
        VelocityControlRequest request = new VelocityControlRequest(0.0)
            .withMotionProfileType(VelocityMotionProfileType.None);
        assertEquals(VelocityMotionProfileType.None, request.MotionProfileType);
    }

    @Test
    void withOutputType() {
        VelocityControlRequest request = new VelocityControlRequest(0.0)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);
        assertEquals(ControlOutputType.TorqueCurrentFOC, request.OutputType);
    }

    @Test
    void chainingMethods() {
        VelocityControlRequest request = new VelocityControlRequest(1.0)
            .withVelocity(2.0)
            .withSlot(1)
            .withUpdateFreqHz(200.0)
            .withUseTimesync(true)
            .withMotionProfileType(VelocityMotionProfileType.None)
            .withOutputType(ControlOutputType.Voltage);

        assertEquals(2.0, request.Velocity);
        assertEquals(1, request.Slot);
        assertEquals(200.0, request.UpdateFreqHz);
        assertTrue(request.UseTimesync);
        assertEquals(VelocityMotionProfileType.None, request.MotionProfileType);
        assertEquals(ControlOutputType.Voltage, request.OutputType);
    }

    @Test
    void toControlRequest_NoneVoltage() {
        VelocityControlRequest request = new VelocityControlRequest(5.0)
            .withMotionProfileType(VelocityMotionProfileType.None)
            .withOutputType(ControlOutputType.Voltage);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(VelocityVoltage.class, result);
    }

    @Test
    void toControlRequest_NoneVoltageFOC() {
        VelocityControlRequest request = new VelocityControlRequest(5.0)
            .withMotionProfileType(VelocityMotionProfileType.None)
            .withOutputType(ControlOutputType.VoltageFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(VelocityVoltage.class, result);
    }

    @Test
    void toControlRequest_NoneTorqueCurrentFOC() {
        VelocityControlRequest request = new VelocityControlRequest(5.0)
            .withMotionProfileType(VelocityMotionProfileType.None)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(VelocityTorqueCurrentFOC.class, result);
    }

    @Test
    void toControlRequest_TrapezoidalVoltage() {
        VelocityControlRequest request = new VelocityControlRequest(5.0)
            .withMotionProfileType(VelocityMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.Voltage);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(MotionMagicVelocityVoltage.class, result);
    }

    @Test
    void toControlRequest_TrapezoidalVoltageFOC() {
        VelocityControlRequest request = new VelocityControlRequest(5.0)
            .withMotionProfileType(VelocityMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.VoltageFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(MotionMagicVelocityVoltage.class, result);
    }

    @Test
    void toControlRequest_TrapezoidalTorqueCurrentFOC() {
        VelocityControlRequest request = new VelocityControlRequest(5.0)
            .withMotionProfileType(VelocityMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);
        
        ControlRequest result = request.toControlRequest();
        assertInstanceOf(MotionMagicVelocityTorqueCurrentFOC.class, result);
    }
}
