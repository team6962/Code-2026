package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import org.junit.jupiter.api.Test;

class WheelMathTest {

  private static final double DELTA = 1e-9;

  @Test
  void toLinear_AngleAndRadius_CalculatesArcLength() {
    Angle angle = Radians.of(2.0);
    Distance radius = Meters.of(0.5);

    Distance result = WheelMath.toLinear(angle, radius);

    assertEquals(1.0, result.in(Meters), DELTA);
  }

  @Test
  void toLinear_ZeroAngle_ReturnsZero() {
    Angle angle = Radians.of(0);
    Distance radius = Meters.of(0.5);

    Distance result = WheelMath.toLinear(angle, radius);

    assertEquals(0.0, result.in(Meters), DELTA);
  }

  @Test
  void toLinear_FullRotation_ReturnsCircumference() {
    Angle angle = Radians.of(2 * Math.PI);
    Distance radius = Meters.of(1.0);

    Distance result = WheelMath.toLinear(angle, radius);

    assertEquals(2 * Math.PI, result.in(Meters), DELTA);
  }

  @Test
  void toAngular_ArcLengthAndRadius_CalculatesAngle() {
    Distance arcLength = Meters.of(1.0);
    Distance radius = Meters.of(0.5);

    Angle result = WheelMath.toAngular(arcLength, radius);

    assertEquals(2.0, result.in(Radians), DELTA);
  }

  @Test
  void toAngular_ZeroArcLength_ReturnsZero() {
    Distance arcLength = Meters.of(0);
    Distance radius = Meters.of(0.5);

    Angle result = WheelMath.toAngular(arcLength, radius);

    assertEquals(0.0, result.in(Radians), DELTA);
  }

  @Test
  void toAngular_CircumferenceArcLength_ReturnsFullRotation() {
    Distance radius = Meters.of(1.0);
    Distance arcLength = Meters.of(2 * Math.PI);

    Angle result = WheelMath.toAngular(arcLength, radius);

    assertEquals(2 * Math.PI, result.in(Radians), DELTA);
  }

  @Test
  void toLinear_AngularVelocityAndRadius_CalculatesLinearVelocity() {
    AngularVelocity angularVelocity = RadiansPerSecond.of(4.0);
    Distance radius = Meters.of(0.5);

    LinearVelocity result = WheelMath.toLinear(angularVelocity, radius);

    assertEquals(2.0, result.in(MetersPerSecond), DELTA);
  }

  @Test
  void toLinear_ZeroAngularVelocity_ReturnsZero() {
    AngularVelocity angularVelocity = RadiansPerSecond.of(0);
    Distance radius = Meters.of(0.5);

    LinearVelocity result = WheelMath.toLinear(angularVelocity, radius);

    assertEquals(0.0, result.in(MetersPerSecond), DELTA);
  }

  @Test
  void toAngular_LinearVelocityAndRadius_CalculatesAngularVelocity() {
    LinearVelocity linearVelocity = MetersPerSecond.of(2.0);
    Distance radius = Meters.of(0.5);

    AngularVelocity result = WheelMath.toAngular(linearVelocity, radius);

    assertEquals(4.0, result.in(RadiansPerSecond), DELTA);
  }

  @Test
  void toAngular_ZeroLinearVelocity_ReturnsZero() {
    LinearVelocity linearVelocity = MetersPerSecond.of(0);
    Distance radius = Meters.of(0.5);

    AngularVelocity result = WheelMath.toAngular(linearVelocity, radius);

    assertEquals(0.0, result.in(RadiansPerSecond), DELTA);
  }

  @Test
  void toLinear_AngularAccelerationAndRadius_CalculatesLinearAcceleration() {
    AngularAcceleration angularAcceleration = RadiansPerSecondPerSecond.of(6.0);
    Distance radius = Meters.of(0.5);

    LinearAcceleration result = WheelMath.toLinear(angularAcceleration, radius);

    assertEquals(3.0, result.in(MetersPerSecondPerSecond), DELTA);
  }

  @Test
  void toLinear_ZeroAngularAcceleration_ReturnsZero() {
    AngularAcceleration angularAcceleration = RadiansPerSecondPerSecond.of(0);
    Distance radius = Meters.of(0.5);

    LinearAcceleration result = WheelMath.toLinear(angularAcceleration, radius);

    assertEquals(0.0, result.in(MetersPerSecondPerSecond), DELTA);
  }

  @Test
  void toAngular_LinearAccelerationAndRadius_CalculatesAngularAcceleration() {
    LinearAcceleration linearAcceleration = MetersPerSecondPerSecond.of(3.0);
    Distance radius = Meters.of(0.5);

    AngularAcceleration result = WheelMath.toAngular(linearAcceleration, radius);

    assertEquals(6.0, result.in(RadiansPerSecondPerSecond), DELTA);
  }

  @Test
  void toAngular_ZeroLinearAcceleration_ReturnsZero() {
    LinearAcceleration linearAcceleration = MetersPerSecondPerSecond.of(0);
    Distance radius = Meters.of(0.5);

    AngularAcceleration result = WheelMath.toAngular(linearAcceleration, radius);

    assertEquals(0.0, result.in(RadiansPerSecondPerSecond), DELTA);
  }

  @Test
  void roundTrip_AngleToLinearToAngle_PreservesValue() {
    Angle original = Radians.of(3.5);
    Distance radius = Meters.of(0.75);

    Distance linear = WheelMath.toLinear(original, radius);
    Angle recovered = WheelMath.toAngular(linear, radius);

    assertEquals(original.in(Radians), recovered.in(Radians), DELTA);
  }

  @Test
  void roundTrip_AngularVelocityToLinearToAngular_PreservesValue() {
    AngularVelocity original = RadiansPerSecond.of(5.0);
    Distance radius = Meters.of(0.75);

    LinearVelocity linear = WheelMath.toLinear(original, radius);
    AngularVelocity recovered = WheelMath.toAngular(linear, radius);

    assertEquals(original.in(RadiansPerSecond), recovered.in(RadiansPerSecond), DELTA);
  }

  @Test
  void roundTrip_AngularAccelerationToLinearToAngular_PreservesValue() {
    AngularAcceleration original = RadiansPerSecondPerSecond.of(8.0);
    Distance radius = Meters.of(0.75);

    LinearAcceleration linear = WheelMath.toLinear(original, radius);
    AngularAcceleration recovered = WheelMath.toAngular(linear, radius);

    assertEquals(
        original.in(RadiansPerSecondPerSecond), recovered.in(RadiansPerSecondPerSecond), DELTA);
  }
}
