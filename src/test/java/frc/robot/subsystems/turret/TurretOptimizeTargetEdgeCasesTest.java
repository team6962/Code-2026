package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.Test;

/**
 * Edge-case tests for {@link Turret#optimizeTarget} using the <b>actual robot turret range</b>
 * ({@code -276°} to {@code 23°}) from {@link TurretConstants}.
 *
 * <p>The actual range is important because:
 *
 * <ul>
 *   <li>It spans 299° (less than 360°), leaving a 61° dead zone.
 *   <li>It is asymmetric around zero, so wrap-around behavior is non-obvious.
 *   <li>optimizeTarget only applies a single ±2π correction, so targets in the dead zone may remain
 *       out of bounds.
 * </ul>
 */
class TurretOptimizeTargetEdgeCasesTest {

  // Actual robot limits from TurretConstants
  private static final Angle MIN = Degrees.of(-276);
  private static final Angle MAX = Degrees.of(23);
  private static final double DELTA = 0.01;

  // Helper: optimize using the real turret range
  private static Angle optimize(double targetDeg, double currentDeg) {
    return Turret.optimizeTarget(Degrees.of(targetDeg), Degrees.of(currentDeg), MIN, MAX);
  }

  // Helper: check result is within the turret's physical limits
  private static void assertInRange(Angle result, String message) {
    double deg = result.in(Degrees);
    assertTrue(
        deg >= -276.0 - DELTA && deg <= 23.0 + DELTA,
        message + " — got " + deg + "°, expected in [-276°, 23°]");
  }

  // ==================== Normal cases within range ====================

  @Test
  void target0_current0_returns0() {
    Angle result = optimize(0, 0);
    assertEquals(0.0, result.in(Degrees), DELTA);
    assertInRange(result, "0° target at 0° current");
  }

  @Test
  void targetNeg90_currentNeg100_returnsNeg90() {
    Angle result = optimize(-90, -100);
    assertEquals(-90.0, result.in(Degrees), DELTA);
    assertInRange(result, "-90° target");
  }

  @Test
  void target20_current10_returns20() {
    // Near the max limit
    Angle result = optimize(20, 10);
    assertEquals(20.0, result.in(Degrees), DELTA);
    assertInRange(result, "20° target near max");
  }

  @Test
  void targetNeg270_currentNeg260_returnsNeg270() {
    // Near the min limit
    Angle result = optimize(-270, -260);
    assertEquals(-270.0, result.in(Degrees), DELTA);
    assertInRange(result, "-270° target near min");
  }

  // ==================== Targets that need wrapping ====================

  @Test
  void target90_current0_wrapsToNeg270() {
    // 90° is outside [-276°, 23°], so it should wrap to 90 - 360 = -270°
    Angle result = optimize(90, 0);
    assertEquals(-270.0, result.in(Degrees), DELTA, "90° should wrap to -270°");
    assertInRange(result, "90° wrapped");
  }

  @Test
  void target180_current0_wrapsToNeg180() {
    // 180° is outside the range, wraps to 180 - 360 = -180° which is in range
    Angle result = optimize(180, 0);
    assertEquals(-180.0, result.in(Degrees), DELTA, "180° should wrap to -180°");
    assertInRange(result, "180° wrapped");
  }

  @Test
  void target350_current0_wrapsToNeg10() {
    // 350° normalizes to -10° (discrete), then continuous near 0 stays at -10°
    Angle result = optimize(350, 0);
    assertEquals(-10.0, result.in(Degrees), DELTA, "350° should normalize to -10°");
    assertInRange(result, "350° normalized");
  }

  // ==================== Dead zone targets (25° to 84°) ====================
  // The dead zone is from MAX (23°) to MIN+360 (84°).
  // Targets in this zone cannot be reached by the turret.

  @Test
  void target50_current0_singleWrapDoesNotFit() {
    // 50° is in the dead zone [23°, 84°].
    // toDiscrete(50) = 50°, toContinuous(50, 0) = 50°
    // 50 > 23 (max), so subtract 360 => -310°
    // -310 < -276 (min), so add 360 => 50° — back where we started!
    //
    // The method's single-pass wrapping can't place this in range.
    // This is a documented limitation, not necessarily a bug,
    // but it means the caller MUST clamp the result.
    Angle result = optimize(50, 0);
    double deg = result.in(Degrees);

    // Document that the result is out of bounds:
    boolean inRange = deg >= -276.0 - DELTA && deg <= 23.0 + DELTA;
    assertFalse(
        inRange,
        "Documented limitation: dead-zone target 50° cannot be mapped into [-276°, 23°], "
            + "got: "
            + deg
            + "°");
  }

  @Test
  void target70_currentNeg270_singleWrapDoesNotFit() {
    // 70° is in the dead zone. From current=-270°:
    // toDiscrete(70) = 70°, toContinuous(70, -270) = 70-360 = -290°
    // -290 < -276, so add 360 => 70° — still out of range
    Angle result = optimize(70, -270);
    double deg = result.in(Degrees);

    boolean inRange = deg >= -276.0 - DELTA && deg <= 23.0 + DELTA;
    assertFalse(
        inRange,
        "Documented limitation: dead-zone target 70° from -270° stays out of range, "
            + "got: "
            + deg
            + "°");
  }

  // ==================== Boundary conditions ====================

  @Test
  void targetAtExactMax_returns23() {
    Angle result = optimize(23, 0);
    assertEquals(23.0, result.in(Degrees), DELTA);
    assertInRange(result, "Exact max angle");
  }

  @Test
  void targetAtExactMin_returnsNeg276() {
    Angle result = optimize(-276, -200);
    assertEquals(-276.0, result.in(Degrees), DELTA);
    assertInRange(result, "Exact min angle");
  }

  @Test
  void targetJustAboveMax_wrapsDown() {
    // 24° is just above max (23°). Should wrap to 24 - 360 = -336°.
    // But -336 < -276 (min), so add 360 => 24° again. Out of range.
    Angle result = optimize(24, 0);
    double deg = result.in(Degrees);

    // This is in the dead zone, so single-pass wrap fails.
    boolean inRange = deg >= -276.0 - DELTA && deg <= 23.0 + DELTA;
    assertFalse(
        inRange,
        "24° (just above max) is in the dead zone and can't be wrapped into range, got: "
            + deg
            + "°");
  }

  @Test
  void targetJustBelowMin_wrapsUp() {
    // -277° is just below min. Discrete: -277 + 360 = 83°.
    // From current=-200°: continuous would be 83 - 360 = -277°.
    // -277 < -276, so add 360 => 83°. But 83 > 23, so subtract 360 => -277 again.
    // Another dead-zone case.
    Angle result = optimize(-277, -200);
    double deg = result.in(Degrees);

    boolean inRange = deg >= -276.0 - DELTA && deg <= 23.0 + DELTA;
    assertFalse(
        inRange,
        "-277° (just below min) is in the dead zone and can't be wrapped, got: " + deg + "°");
  }

  // ==================== Practical shooter scenarios ====================

  @Test
  void shooterPointingBackward_optimizesShortPath() {
    // Shooter sees target at 170° field-relative (behind the robot).
    // Discrete: 170°. Current: -10°.
    // Continuous near -10: 170° (distance 180°). That's > 23, so wrap to -190°.
    // -190° is in range [-276, 23]. That's 180° of travel from -10°.
    Angle result = optimize(170, -10);
    assertEquals(-190.0, result.in(Degrees), DELTA);
    assertInRange(result, "Backward target wraps to -190°");
  }

  @Test
  void shooterPointingForward_noWrapNeeded() {
    // Target at -15° field-relative, current at 5°.
    Angle result = optimize(-15, 5);
    assertEquals(-15.0, result.in(Degrees), DELTA);
    assertInRange(result, "Forward target stays at -15°");
  }

  @Test
  void shooterFromFarNegative_optimizesToShortPath() {
    // Current at -250°, target at -10°.
    // Should pick -10° (240° travel) not -370° (-10 - 360 = -370, out of range).
    Angle result = optimize(-10, -250);
    assertEquals(-10.0, result.in(Degrees), DELTA);
    assertInRange(result, "From far negative, optimize to -10°");
  }
}
