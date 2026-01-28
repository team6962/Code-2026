package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.MassUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import org.junit.jupiter.api.Test;

class MeasureUtilTest {

  private static final double DELTA = 1e-9;

  // ==================== toAngle() tests ====================

  @Test
  void toAngle_FromRadians_ReturnsCorrectAngle() {
    var measure = Radians.of(Math.PI);
    Angle result = MeasureUtil.toAngle(measure);
    assertEquals(Math.PI, result.in(Radians), DELTA);
  }

  @Test
  void toAngle_FromDegrees_ReturnsCorrectAngle() {
    var measure = Degrees.of(180);
    Angle result = MeasureUtil.toAngle(measure);
    assertEquals(Math.PI, result.in(Radians), DELTA);
  }

  @Test
  void toAngle_Zero_ReturnsZero() {
    var measure = Radians.of(0);
    Angle result = MeasureUtil.toAngle(measure);
    assertEquals(0.0, result.in(Radians), DELTA);
  }

  @Test
  void toAngle_Negative_ReturnsNegative() {
    var measure = Radians.of(-1.5);
    Angle result = MeasureUtil.toAngle(measure);
    assertEquals(-1.5, result.in(Radians), DELTA);
  }

  // ==================== toAngularVelocity() tests ====================

  @Test
  @SuppressWarnings("unchecked")
  void toAngularVelocity_FromRadiansPerSecond_ReturnsCorrectVelocity() {
    Measure<PerUnit<AngleUnit, TimeUnit>> measure =
        (Measure<PerUnit<AngleUnit, TimeUnit>>) (Measure<?>) RadiansPerSecond.of(5.0);
    AngularVelocity result = MeasureUtil.toAngularVelocity(measure);
    assertEquals(5.0, result.in(RadiansPerSecond), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void toAngularVelocity_Zero_ReturnsZero() {
    Measure<PerUnit<AngleUnit, TimeUnit>> measure =
        (Measure<PerUnit<AngleUnit, TimeUnit>>) (Measure<?>) RadiansPerSecond.of(0);
    AngularVelocity result = MeasureUtil.toAngularVelocity(measure);
    assertEquals(0.0, result.in(RadiansPerSecond), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void toAngularVelocity_Negative_ReturnsNegative() {
    Measure<PerUnit<AngleUnit, TimeUnit>> measure =
        (Measure<PerUnit<AngleUnit, TimeUnit>>) (Measure<?>) RadiansPerSecond.of(-3.14);
    AngularVelocity result = MeasureUtil.toAngularVelocity(measure);
    assertEquals(-3.14, result.in(RadiansPerSecond), DELTA);
  }

  // ==================== toAngularAcceleration() tests ====================

  @Test
  @SuppressWarnings("unchecked")
  void toAngularAcceleration_FromRadiansPerSecondSquared_ReturnsCorrectAcceleration() {
    Measure<PerUnit<PerUnit<AngleUnit, TimeUnit>, TimeUnit>> measure =
        (Measure<PerUnit<PerUnit<AngleUnit, TimeUnit>, TimeUnit>>)
            (Measure<?>) RadiansPerSecondPerSecond.of(10.0);
    AngularAcceleration result = MeasureUtil.toAngularAcceleration(measure);
    assertEquals(10.0, result.in(RadiansPerSecondPerSecond), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void toAngularAcceleration_Zero_ReturnsZero() {
    Measure<PerUnit<PerUnit<AngleUnit, TimeUnit>, TimeUnit>> measure =
        (Measure<PerUnit<PerUnit<AngleUnit, TimeUnit>, TimeUnit>>)
            (Measure<?>) RadiansPerSecondPerSecond.of(0);
    AngularAcceleration result = MeasureUtil.toAngularAcceleration(measure);
    assertEquals(0.0, result.in(RadiansPerSecondPerSecond), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void toAngularAcceleration_Negative_ReturnsNegative() {
    Measure<PerUnit<PerUnit<AngleUnit, TimeUnit>, TimeUnit>> measure =
        (Measure<PerUnit<PerUnit<AngleUnit, TimeUnit>, TimeUnit>>)
            (Measure<?>) RadiansPerSecondPerSecond.of(-7.5);
    AngularAcceleration result = MeasureUtil.toAngularAcceleration(measure);
    assertEquals(-7.5, result.in(RadiansPerSecondPerSecond), DELTA);
  }

  // ==================== toDistance() tests ====================

  @Test
  void toDistance_FromMeters_ReturnsCorrectDistance() {
    var measure = Meters.of(2.5);
    Distance result = MeasureUtil.toDistance(measure);
    assertEquals(2.5, result.in(Meters), DELTA);
  }

  @Test
  void toDistance_Zero_ReturnsZero() {
    var measure = Meters.of(0);
    Distance result = MeasureUtil.toDistance(measure);
    assertEquals(0.0, result.in(Meters), DELTA);
  }

  @Test
  void toDistance_Negative_ReturnsNegative() {
    var measure = Meters.of(-1.0);
    Distance result = MeasureUtil.toDistance(measure);
    assertEquals(-1.0, result.in(Meters), DELTA);
  }

  // ==================== toLinearVelocity() tests ====================

  @Test
  @SuppressWarnings("unchecked")
  void toLinearVelocity_FromMetersPerSecond_ReturnsCorrectVelocity() {
    Measure<PerUnit<DistanceUnit, TimeUnit>> measure =
        (Measure<PerUnit<DistanceUnit, TimeUnit>>) (Measure<?>) MetersPerSecond.of(15.0);
    LinearVelocity result = MeasureUtil.toLinearVelocity(measure);
    assertEquals(15.0, result.in(MetersPerSecond), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void toLinearVelocity_Zero_ReturnsZero() {
    Measure<PerUnit<DistanceUnit, TimeUnit>> measure =
        (Measure<PerUnit<DistanceUnit, TimeUnit>>) (Measure<?>) MetersPerSecond.of(0);
    LinearVelocity result = MeasureUtil.toLinearVelocity(measure);
    assertEquals(0.0, result.in(MetersPerSecond), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void toLinearVelocity_Negative_ReturnsNegative() {
    Measure<PerUnit<DistanceUnit, TimeUnit>> measure =
        (Measure<PerUnit<DistanceUnit, TimeUnit>>) (Measure<?>) MetersPerSecond.of(-8.0);
    LinearVelocity result = MeasureUtil.toLinearVelocity(measure);
    assertEquals(-8.0, result.in(MetersPerSecond), DELTA);
  }

  // ==================== toLinearAcceleration() tests ====================

  @Test
  @SuppressWarnings("unchecked")
  void toLinearAcceleration_FromMetersPerSecondSquared_ReturnsCorrectAcceleration() {
    Measure<PerUnit<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>> measure =
        (Measure<PerUnit<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>>)
            (Measure<?>) MetersPerSecondPerSecond.of(9.81);
    LinearAcceleration result = MeasureUtil.toLinearAcceleration(measure);
    assertEquals(9.81, result.in(MetersPerSecondPerSecond), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void toLinearAcceleration_Zero_ReturnsZero() {
    Measure<PerUnit<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>> measure =
        (Measure<PerUnit<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>>)
            (Measure<?>) MetersPerSecondPerSecond.of(0);
    LinearAcceleration result = MeasureUtil.toLinearAcceleration(measure);
    assertEquals(0.0, result.in(MetersPerSecondPerSecond), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void toLinearAcceleration_Negative_ReturnsNegative() {
    Measure<PerUnit<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>> measure =
        (Measure<PerUnit<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>>)
            (Measure<?>) MetersPerSecondPerSecond.of(-5.0);
    LinearAcceleration result = MeasureUtil.toLinearAcceleration(measure);
    assertEquals(-5.0, result.in(MetersPerSecondPerSecond), DELTA);
  }

  // ==================== toMomentOfInertia() tests ====================

  @Test
  @SuppressWarnings("unchecked")
  void toMomentOfInertia_FromKilogramSquareMeters_ReturnsCorrectMOI() {
    Measure<MultUnit<MultUnit<DistanceUnit, DistanceUnit>, MassUnit>> measure =
        (Measure<MultUnit<MultUnit<DistanceUnit, DistanceUnit>, MassUnit>>)
            (Measure<?>) KilogramSquareMeters.of(1.5);
    MomentOfInertia result = MeasureUtil.toMomentOfInertia(measure);
    assertEquals(1.5, result.in(KilogramSquareMeters), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void toMomentOfInertia_Zero_ReturnsZero() {
    Measure<MultUnit<MultUnit<DistanceUnit, DistanceUnit>, MassUnit>> measure =
        (Measure<MultUnit<MultUnit<DistanceUnit, DistanceUnit>, MassUnit>>)
            (Measure<?>) KilogramSquareMeters.of(0);
    MomentOfInertia result = MeasureUtil.toMomentOfInertia(measure);
    assertEquals(0.0, result.in(KilogramSquareMeters), DELTA);
  }

  @Test
  @SuppressWarnings("unchecked")
  void toMomentOfInertia_LargeValue_ReturnsCorrectMOI() {
    Measure<MultUnit<MultUnit<DistanceUnit, DistanceUnit>, MassUnit>> measure =
        (Measure<MultUnit<MultUnit<DistanceUnit, DistanceUnit>, MassUnit>>)
            (Measure<?>) KilogramSquareMeters.of(100.0);
    MomentOfInertia result = MeasureUtil.toMomentOfInertia(measure);
    assertEquals(100.0, result.in(KilogramSquareMeters), DELTA);
  }
}
