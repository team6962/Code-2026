package com.team6962.lib.phoenix.control;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static org.junit.jupiter.api.Assertions.*;

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
import org.junit.jupiter.api.Test;

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
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(5.0, 10.0, 20.0, 30.0);
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
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(0.0).withPosition(10.0);
    assertEquals(10.0, request.Position);
  }

  @Test
  void withPositionAngle() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(0.0).withPosition(Rotations.of(7.5));
    assertEquals(7.5, request.Position, 0.0001);
  }

  @Test
  void withVelocityDouble() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(0.0).withVelocity(15.0);
    assertEquals(15.0, request.Velocity);
  }

  @Test
  void withVelocityAngularVelocity() {
    AngularVelocity velocity = RotationsPerSecond.of(12.5);
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(0.0).withVelocity(velocity);
    assertEquals(12.5, request.Velocity, 0.0001);
  }

  @Test
  void withAccelerationDouble() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(0.0).withAcceleration(25.0);
    assertEquals(25.0, request.Acceleration);
  }

  @Test
  void withAccelerationAngularAcceleration() {
    AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(18.5);
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(0.0).withAcceleration(acceleration);
    assertEquals(18.5, request.Acceleration, 0.0001);
  }

  @Test
  void withJerkDouble() {
    DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0).withJerk(35.0);
    assertEquals(35.0, request.Jerk);
  }

  @Test
  @SuppressWarnings("unchecked")
  void withJerkMeasure() {
    // Jerk is in rotations per second cubed, which is (rotations per second squared) per second
    Measure<PerUnit<AngularAccelerationUnit, TimeUnit>> jerk =
        (Measure<PerUnit<AngularAccelerationUnit, TimeUnit>>)
            (Measure<?>) RotationsPerSecondPerSecond.per(Second).of(42.5);
    DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0).withJerk(jerk);
    assertEquals(42.5, request.Jerk, 0.0001);
  }

  @Test
  void withSlot() {
    DynamicPositionControlRequest request = new DynamicPositionControlRequest(0.0).withSlot(2);
    assertEquals(2, request.Slot);
  }

  @Test
  void withUpdateFreqHz() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(0.0).withUpdateFreqHz(100.0);
    assertEquals(100.0, request.UpdateFreqHz);
  }

  @Test
  void withUseTimesync() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(0.0).withUseTimesync(true);
    assertTrue(request.UseTimesync);
  }

  @Test
  void withMotionProfileType() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(0.0)
            .withMotionProfileType(DynamicPositionMotionProfileType.Trapezoidal);
    assertEquals(DynamicPositionMotionProfileType.Trapezoidal, request.MotionProfileType);
  }

  @Test
  void withOutputType() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(0.0).withOutputType(ControlOutputType.TorqueCurrentFOC);
    assertEquals(ControlOutputType.TorqueCurrentFOC, request.OutputType);
  }

  @Test
  void chainingMethods() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(1.0)
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
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(5.0)
            .withVelocity(10.0)
            .withAcceleration(20.0)
            .withJerk(30.0)
            .withSlot(1)
            .withUpdateFreqHz(100.0)
            .withUseTimesync(true)
            .withMotionProfileType(DynamicPositionMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.Voltage);

    ControlRequest result = request.toControlRequest();
    assertInstanceOf(DynamicMotionMagicVoltage.class, result);
    DynamicMotionMagicVoltage typed = (DynamicMotionMagicVoltage) result;
    assertEquals(5.0, typed.Position);
    assertEquals(10.0, typed.Velocity);
    assertEquals(20.0, typed.Acceleration);
    assertEquals(30.0, typed.Jerk);
    assertEquals(1, typed.Slot);
    assertEquals(100.0, typed.UpdateFreqHz);
    assertTrue(typed.UseTimesync);
    assertFalse(typed.EnableFOC);
  }

  @Test
  void toControlRequest_TrapezoidalVoltageFOC() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(5.0)
            .withVelocity(15.0)
            .withAcceleration(25.0)
            .withJerk(35.0)
            .withSlot(2)
            .withUpdateFreqHz(200.0)
            .withUseTimesync(false)
            .withMotionProfileType(DynamicPositionMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.VoltageFOC);

    ControlRequest result = request.toControlRequest();
    assertInstanceOf(DynamicMotionMagicVoltage.class, result);
    DynamicMotionMagicVoltage typed = (DynamicMotionMagicVoltage) result;
    assertEquals(5.0, typed.Position);
    assertEquals(15.0, typed.Velocity);
    assertEquals(25.0, typed.Acceleration);
    assertEquals(35.0, typed.Jerk);
    assertEquals(2, typed.Slot);
    assertEquals(200.0, typed.UpdateFreqHz);
    assertFalse(typed.UseTimesync);
    assertTrue(typed.EnableFOC);
  }

  @Test
  void toControlRequest_TrapezoidalTorqueCurrentFOC() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(5.0)
            .withVelocity(12.0)
            .withAcceleration(22.0)
            .withJerk(32.0)
            .withSlot(0)
            .withUpdateFreqHz(150.0)
            .withUseTimesync(true)
            .withMotionProfileType(DynamicPositionMotionProfileType.Trapezoidal)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);

    ControlRequest result = request.toControlRequest();
    assertInstanceOf(DynamicMotionMagicTorqueCurrentFOC.class, result);
    DynamicMotionMagicTorqueCurrentFOC typed = (DynamicMotionMagicTorqueCurrentFOC) result;
    assertEquals(5.0, typed.Position);
    assertEquals(12.0, typed.Velocity);
    assertEquals(22.0, typed.Acceleration);
    assertEquals(32.0, typed.Jerk);
    assertEquals(0, typed.Slot);
    assertEquals(150.0, typed.UpdateFreqHz);
    assertTrue(typed.UseTimesync);
  }

  @Test
  void toControlRequest_ExponentialVoltage() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(5.0)
            .withSlot(1)
            .withUpdateFreqHz(80.0)
            .withUseTimesync(true)
            .withMotionProfileType(DynamicPositionMotionProfileType.Exponential)
            .withOutputType(ControlOutputType.Voltage);

    ControlRequest result = request.toControlRequest();
    assertInstanceOf(DynamicMotionMagicExpoVoltage.class, result);
    DynamicMotionMagicExpoVoltage typed = (DynamicMotionMagicExpoVoltage) result;
    assertEquals(5.0, typed.Position);
    assertEquals(1, typed.Slot);
    assertEquals(80.0, typed.UpdateFreqHz);
    assertTrue(typed.UseTimesync);
    assertFalse(typed.EnableFOC);
  }

  @Test
  void toControlRequest_ExponentialVoltageFOC() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(5.0)
            .withSlot(0)
            .withUpdateFreqHz(60.0)
            .withUseTimesync(false)
            .withMotionProfileType(DynamicPositionMotionProfileType.Exponential)
            .withOutputType(ControlOutputType.VoltageFOC);

    ControlRequest result = request.toControlRequest();
    assertInstanceOf(DynamicMotionMagicExpoVoltage.class, result);
    DynamicMotionMagicExpoVoltage typed = (DynamicMotionMagicExpoVoltage) result;
    assertEquals(5.0, typed.Position);
    assertEquals(0, typed.Slot);
    assertEquals(60.0, typed.UpdateFreqHz);
    assertFalse(typed.UseTimesync);
    assertTrue(typed.EnableFOC);
  }

  @Test
  void toControlRequest_ExponentialTorqueCurrentFOC() {
    DynamicPositionControlRequest request =
        new DynamicPositionControlRequest(5.0)
            .withSlot(2)
            .withUpdateFreqHz(120.0)
            .withUseTimesync(true)
            .withMotionProfileType(DynamicPositionMotionProfileType.Exponential)
            .withOutputType(ControlOutputType.TorqueCurrentFOC);

    ControlRequest result = request.toControlRequest();
    assertInstanceOf(DynamicMotionMagicExpoTorqueCurrentFOC.class, result);
    DynamicMotionMagicExpoTorqueCurrentFOC typed = (DynamicMotionMagicExpoTorqueCurrentFOC) result;
    assertEquals(5.0, typed.Position);
    assertEquals(2, typed.Slot);
    assertEquals(120.0, typed.UpdateFreqHz);
    assertTrue(typed.UseTimesync);
  }

  @Test
  void toControlRequest_NullMotionProfileType_ThrowsException() {
    DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0);
    request.MotionProfileType = null;

    assertThrows(IllegalStateException.class, () -> request.toControlRequest());
  }

  @Test
  void toControlRequest_NullOutputType_ThrowsException() {
    DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0);
    request.OutputType = null;

    assertThrows(IllegalStateException.class, () -> request.toControlRequest());
  }

  @Test
  void toControlRequest_BothNull_ThrowsException() {
    DynamicPositionControlRequest request = new DynamicPositionControlRequest(5.0);
    request.MotionProfileType = null;
    request.OutputType = null;

    assertThrows(IllegalStateException.class, () -> request.toControlRequest());
  }
}
