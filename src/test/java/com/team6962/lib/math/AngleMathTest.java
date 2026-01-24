package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.Test;

class AngleMathTest {

  private static final double DELTA = 1e-9;

  // ==================== toDiscrete() tests ====================

  @Test
  void toDiscrete_ZeroAngle_ReturnsZero() {
    Angle result = AngleMath.toDiscrete(Radians.of(0));
    assertEquals(0.0, result.in(Radians), DELTA);
  }

  @Test
  void toDiscrete_PositiveAngleWithinRange_ReturnsSameAngle() {
    Angle result = AngleMath.toDiscrete(Degrees.of(45));
    assertEquals(45.0, result.in(Degrees), DELTA);
  }

  @Test
  void toDiscrete_NegativeAngleWithinRange_ReturnsSameAngle() {
    Angle result = AngleMath.toDiscrete(Degrees.of(-90));
    assertEquals(-90.0, result.in(Degrees), DELTA);
  }

  @Test
  void toDiscrete_Exactly180Degrees_Returns180() {
    Angle result = AngleMath.toDiscrete(Degrees.of(180));
    // 180 degrees is at the boundary, could be +180 or -180
    assertEquals(Math.PI, Math.abs(result.in(Radians)), DELTA);
  }

  @Test
  void toDiscrete_270Degrees_ReturnsMinus90() {
    Angle result = AngleMath.toDiscrete(Degrees.of(270));
    assertEquals(-90.0, result.in(Degrees), DELTA);
  }

  @Test
  void toDiscrete_360Degrees_ReturnsZero() {
    Angle result = AngleMath.toDiscrete(Degrees.of(360));
    assertEquals(0.0, result.in(Degrees), DELTA);
  }

  @Test
  void toDiscrete_450Degrees_Returns90() {
    Angle result = AngleMath.toDiscrete(Degrees.of(450));
    assertEquals(90.0, result.in(Degrees), DELTA);
  }

  @Test
  void toDiscrete_Minus270Degrees_Returns90() {
    Angle result = AngleMath.toDiscrete(Degrees.of(-270));
    assertEquals(90.0, result.in(Degrees), DELTA);
  }

  @Test
  void toDiscrete_LargePositiveAngle_WrapsCorrectly() {
    // 720 + 45 = 765 degrees should wrap to 45 degrees
    Angle result = AngleMath.toDiscrete(Degrees.of(765));
    assertEquals(45.0, result.in(Degrees), DELTA);
  }

  @Test
  void toDiscrete_LargeNegativeAngle_WrapsCorrectly() {
    // -720 - 45 = -765 degrees should wrap to -45 degrees
    Angle result = AngleMath.toDiscrete(Degrees.of(-765));
    assertEquals(-45.0, result.in(Degrees), DELTA);
  }

  @Test
  void toDiscrete_OneRotation_ReturnsZero() {
    Angle result = AngleMath.toDiscrete(Rotations.of(1));
    assertEquals(0.0, result.in(Rotations), DELTA);
  }

  @Test
  void toDiscrete_HalfRotation_ReturnsHalfRotation() {
    Angle result = AngleMath.toDiscrete(Rotations.of(0.5));
    assertEquals(0.5, Math.abs(result.in(Rotations)), DELTA);
  }

  // ==================== toContinuous() tests ====================

  @Test
  void toContinuous_DiscreteZero_NearbyZero_ReturnsZero() {
    Angle result = AngleMath.toContinuous(Radians.of(0), Radians.of(0));
    assertEquals(0.0, result.in(Radians), DELTA);
  }

  @Test
  void toContinuous_Discrete45_NearbyZero_Returns45() {
    Angle result = AngleMath.toContinuous(Degrees.of(45), Degrees.of(0));
    assertEquals(45.0, result.in(Degrees), DELTA);
  }

  @Test
  void toContinuous_Discrete45_Nearby360_Returns405() {
    Angle result = AngleMath.toContinuous(Degrees.of(45), Degrees.of(360));
    assertEquals(405.0, result.in(Degrees), DELTA);
  }

  @Test
  void toContinuous_Discrete45_NearbyMinus360_ReturnsMinus315() {
    Angle result = AngleMath.toContinuous(Degrees.of(45), Degrees.of(-360));
    assertEquals(-315.0, result.in(Degrees), DELTA);
  }

  @Test
  void toContinuous_DiscreteMinus90_Nearby270_Returns270() {
    // -90 degrees discrete, nearby 270 continuous
    // The closest continuous equivalent to -90 that's near 270 is 270
    Angle result = AngleMath.toContinuous(Degrees.of(-90), Degrees.of(270));
    assertEquals(270.0, result.in(Degrees), DELTA);
  }

  @Test
  void toContinuous_Discrete90_Nearby720_Returns810() {
    // 90 degrees discrete, nearby 720 continuous (2 full rotations)
    // The closest equivalent is 720 + 90 = 810
    Angle result = AngleMath.toContinuous(Degrees.of(90), Degrees.of(720));
    assertEquals(810.0, result.in(Degrees), DELTA);
  }

  @Test
  void toContinuous_DiscreteZero_Nearby180_ReturnsZeroOrWrapped() {
    // 0 degrees discrete, nearby 180 continuous
    // Nearest multiples of 360 to 180 are 0 and 360
    // round(180/360) = round(0.5) = 0 (or 1, depending on rounding)
    // So result should be 0 or 360
    Angle result = AngleMath.toContinuous(Degrees.of(0), Degrees.of(180));
    // Math.round(0.5) = 1 in Java, so 1 * 360 = 360
    assertEquals(360.0, result.in(Degrees), DELTA);
  }

  @Test
  void toContinuous_RoundTripWithToDiscrete() {
    // Test that toContinuous can recover a continuous angle after toDiscrete
    Angle originalContinuous = Degrees.of(450);
    Angle discrete = AngleMath.toDiscrete(originalContinuous);
    Angle recovered = AngleMath.toContinuous(discrete, originalContinuous);
    assertEquals(originalContinuous.in(Degrees), recovered.in(Degrees), DELTA);
  }

  @Test
  void toContinuous_RoundTripNegative() {
    // Test round trip with negative continuous angle
    Angle originalContinuous = Degrees.of(-450);
    Angle discrete = AngleMath.toDiscrete(originalContinuous);
    Angle recovered = AngleMath.toContinuous(discrete, originalContinuous);
    assertEquals(originalContinuous.in(Degrees), recovered.in(Degrees), DELTA);
  }
}
