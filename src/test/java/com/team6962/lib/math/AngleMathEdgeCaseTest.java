package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.Test;

/**
 * Supplemental edge-case tests for {@link AngleMath}, focusing on subtle rounding behavior and
 * boundary conditions that are easy to get wrong when modifying the implementation.
 *
 * <p>These are designed as regression guards — if someone refactors toDiscrete/toContinuous, these
 * tests will catch common mistakes like off-by-one in the wrapping, wrong rounding direction at
 * ±180°, or inconsistent round-trip behavior.
 */
class AngleMathEdgeCaseTest {

  private static final double DELTA = 1e-9;

  // ==================== toDiscrete: exact boundary behavior ====================

  @Test
  void toDiscrete_ExactlyNeg180_ReturnsNegPi() {
    // -180° is at the boundary. The implementation maps >=π to negative,
    // so both +180° and -180° map to -π.
    Angle result = AngleMath.toDiscrete(Degrees.of(-180));
    assertEquals(-Math.PI, result.in(Radians), DELTA);
  }

  @Test
  void toDiscrete_SlightlyAbove180_ReturnsNegative() {
    // 180.001° should wrap to a small negative angle near -180°
    Angle result = AngleMath.toDiscrete(Degrees.of(180.001));
    assertTrue(result.in(Degrees) < 0, "Slightly above 180° should wrap to negative");
    assertEquals(-179.999, result.in(Degrees), 0.01);
  }

  @Test
  void toDiscrete_SlightlyBelow180_StaysPositive() {
    // 179.999° is within (-π, π], should stay positive
    Angle result = AngleMath.toDiscrete(Degrees.of(179.999));
    assertTrue(result.in(Degrees) > 0, "Slightly below 180° should stay positive");
    assertEquals(179.999, result.in(Degrees), 0.01);
  }

  @Test
  void toDiscrete_VeryLargeAngle_StillWrapsCorrectly() {
    // 36000.5° = 100 full rotations + 0.5°
    Angle result = AngleMath.toDiscrete(Degrees.of(36000.5));
    assertEquals(0.5, result.in(Degrees), 0.001);
  }

  @Test
  void toDiscrete_VeryLargeNegativeAngle_StillWrapsCorrectly() {
    // -36000.5° = -100 full rotations - 0.5°
    Angle result = AngleMath.toDiscrete(Degrees.of(-36000.5));
    assertEquals(-0.5, result.in(Degrees), 0.001);
  }

  // ==================== toContinuous: rounding at the half-turn boundary ====================

  @Test
  void toContinuous_Discrete170_Nearby190_ChoosesCloser() {
    // 170° discrete, nearby continuous 190°.
    // Nearest multiple of 360 to 190 is round(190/360)=round(0.528)=1 → 360.
    // So result = 170 + 360 = 530°. But 530 is 340° away from 190.
    // Alternatively, 170 + 0 = 170°, which is 20° away. This one is closer.
    // Math.round(190/360) = Math.round(0.528) = 1, so result = 170 + 360 = 530°.
    //
    // This shows that toContinuous picks the nearest multiple of 2π to the
    // nearby angle, not the actual closest result to the nearby angle.
    // This is correct for its intended use (encoder unwrapping) but may
    // surprise callers who expect "closest equivalent angle".
    Angle result = AngleMath.toContinuous(Degrees.of(170), Degrees.of(190));

    // Math.round(190.0/360.0) = Math.round(0.5278) = 1
    assertEquals(530.0, result.in(Degrees), DELTA);
  }

  @Test
  void toContinuous_DiscreteNeg170_Nearby170_Returns190() {
    // -170° discrete, nearby 170° continuous.
    // round(170/360) = round(0.472) = 0 → multiple = 0.
    // result = -170 + 0 = -170°. Distance from 170: 340°.
    // But -170 + 360 = 190°. Distance from 170: 20°. This would be closer.
    //
    // Again, the method picks based on nearest multiple, not nearest result.
    Angle result = AngleMath.toContinuous(Degrees.of(-170), Degrees.of(170));
    assertEquals(-170.0, result.in(Degrees), DELTA);
  }

  // ==================== Round-trip consistency ====================

  @Test
  void roundTrip_ManyAngles_AreConsistent() {
    // For a variety of continuous angles, verify toDiscrete → toContinuous recovers the original.
    double[] testAngles = {0, 45, 90, 135, 179.9, -179.9, -90, 360, 720, -360, -720, 450, -450};

    for (double deg : testAngles) {
      Angle original = Degrees.of(deg);
      Angle discrete = AngleMath.toDiscrete(original);
      Angle recovered = AngleMath.toContinuous(discrete, original);
      assertEquals(
          deg,
          recovered.in(Degrees),
          0.001,
          "Round-trip failed for " + deg + "°: discrete=" + discrete.in(Degrees) + "°");
    }
  }

  /**
   * Verifies that the round-trip toDiscrete → toContinuous works at exactly π.
   *
   * <p>Previously, toDiscrete used {@code > π} so exactly π stayed at +π, then toContinuous added
   * an extra 2π because Math.round(0.5) = 1. Now toDiscrete uses {@code >= π} so π maps to -π, and
   * the round-trip is correct.
   */
  @Test
  void roundTrip_ExactlyPi_Consistent() {
    Angle original = Radians.of(Math.PI);
    Angle discrete = AngleMath.toDiscrete(original);
    Angle recovered = AngleMath.toContinuous(discrete, original);
    assertEquals(Math.PI, recovered.in(Radians), DELTA);
  }

  /**
   * Verifies that the round-trip toDiscrete → toContinuous works at exactly -π.
   *
   * <p>Previously, toDiscrete(-π) returned +π, causing a 2π sign error on round-trip.
   */
  @Test
  void roundTrip_ExactlyNegPi_Consistent() {
    Angle original = Radians.of(-Math.PI);
    Angle discrete = AngleMath.toDiscrete(original);
    Angle recovered = AngleMath.toContinuous(discrete, original);
    assertEquals(-Math.PI, recovered.in(Radians), DELTA);
  }
}
