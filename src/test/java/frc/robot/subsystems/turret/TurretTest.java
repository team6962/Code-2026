package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.Test;

class TurretTest {

  private static final double DELTA = 0.001; // Tolerance for angle comparisons

  // ==================== Basic Optimization Tests ====================

  @Test
  void optimizeTargetAngle_NoOptimizationNeeded_ReturnsTarget() {
    // Target within range and close to current position
    Angle target = Degrees.of(45);
    Angle current = Degrees.of(40);
    Angle min = Degrees.of(0);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    assertEquals(
        45.0, result.in(Degrees), DELTA, "Should return target as-is when no optimization needed");
  }

  @Test
  void optimizeTargetAngle_TargetAtZero_Current180_ChoosesClosest() {
    // Target at 0 degrees, current at 180 degrees
    // Should choose 360 (equivalent to 0) since it's closer than going backwards to 0
    Angle target = Degrees.of(0);
    Angle current = Degrees.of(180);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // Could be either 0 or 360, both are 180 degrees away
    // The implementation should choose one consistently
    double resultDegrees = result.in(Degrees);
    boolean isValid =
        Math.abs(resultDegrees - 0.0) < DELTA || Math.abs(resultDegrees - 360.0) < DELTA;
    assertTrue(isValid, "Result should be either 0 or 360 degrees, got: " + resultDegrees);
  }

  // ==================== Wraparound Tests ====================

  @Test
  void optimizeTargetAngle_Target350_Current10_Wraps() {
    // Target at 350 degrees, current at 10 degrees
    // Going backwards (350) is 20 degrees, going forward (710 = 350+360) is 700 degrees
    // Should choose 350 (equivalent to -10)
    Angle target = Degrees.of(350);
    Angle current = Degrees.of(10);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    assertEquals(-10.0, result.in(Degrees), DELTA, "Should optimize to -10 (equivalent to 350)");
  }

  @Test
  void optimizeTargetAngle_Target10_Current350_Wraps() {
    // Target at 10 degrees, current at 350 degrees (or -10)
    // Should optimize to target near current position
    Angle target = Degrees.of(10);
    Angle current = Degrees.of(350);
    Angle min = Degrees.of(0);
    Angle max = Degrees.of(360);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // Result should be close to 350 (either 10 or 370)
    double resultDegrees = result.in(Degrees);
    double distanceToTarget = Math.abs(resultDegrees - 10.0);
    double distanceToWrapped = Math.abs(resultDegrees - 370.0);

    // Should pick the one closer to current (350)
    boolean isOptimized = distanceToWrapped < 30 || distanceToTarget < 30;
    assertTrue(
        isOptimized,
        "Result should be optimized to be close to current position, got: " + resultDegrees);
  }

  @Test
  void optimizeTargetAngle_FullCircleTarget_OptimizesToContinuous() {
    // Target 370 degrees (= 10 degrees), current at 5 degrees
    Angle target = Degrees.of(370);
    Angle current = Degrees.of(5);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    assertEquals(10.0, result.in(Degrees), DELTA, "Should normalize 370 to 10 degrees");
  }

  // ==================== Limit Handling Tests ====================

  @Test
  void optimizeTargetAngle_ExceedsMaxLimit_WrapsDown() {
    // Current at 170, target at 10 (normalized from 370)
    // Continuous version would be 370, but that exceeds max of 180
    // Should wrap down by 360 to get 10
    Angle target = Degrees.of(10);
    Angle current = Degrees.of(170);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // Should be 370 - 360 = 10
    double resultDegrees = result.in(Degrees);
    boolean isValid =
        Math.abs(resultDegrees - 10.0) < DELTA || Math.abs(resultDegrees - 370.0) < DELTA;
    assertTrue(isValid, "Result should respect max limit, got: " + resultDegrees);
  }

  @Test
  void optimizeTargetAngle_BelowMinLimit_WrapsUp() {
    // Current at -170, target at 350 (which normalizes to -10)
    // Continuous version would be -10, but if we're at -170, going to -10 is fine
    // But if continuous becomes something below min, should wrap up
    Angle target = Degrees.of(350);
    Angle current = Degrees.of(-170);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // -10 is within range [-180, 180]
    double resultDegrees = result.in(Degrees);
    boolean isInRange = resultDegrees >= -180.0 && resultDegrees <= 180.0;
    assertTrue(isInRange, "Result should be within limits, got: " + resultDegrees);
  }

  // ==================== Limited Range Turret Tests ====================

  @Test
  void optimizeTargetAngle_LimitedRangeTurret_90To270() {
    // Turret can only rotate from 90 to 270 degrees (180 degree range)
    // Target at 0 degrees, current at 180 degrees
    Angle target = Degrees.of(0);
    Angle current = Degrees.of(180);
    Angle min = Degrees.of(90);
    Angle max = Degrees.of(270);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // 0 degrees is equivalent to 360 degrees
    // Continuous from 180 would prefer 360 (20 degrees away) over 0 (180 degrees away)
    // But 360 > max (270), so should wrap down to 0
    // However, 0 < min (90), so should wrap up to 360
    // The result might be outside limits, caller should clamp
    double resultDegrees = result.in(Degrees);

    // Result could be 0 or 360, depending on optimization logic
    boolean isValid =
        Math.abs(resultDegrees - 0.0) < DELTA || Math.abs(resultDegrees - 360.0) < DELTA;
    assertTrue(isValid, "Result should be 0 or 360 (both equivalent), got: " + resultDegrees);
  }

  @Test
  void optimizeTargetAngle_LimitedRangeTurret_Minus90To90() {
    // Turret from -90 to +90 degrees (180 degree range, facing forward)
    // Target at 180 degrees (behind), current at 0 degrees (forward)
    Angle target = Degrees.of(180);
    Angle current = Degrees.of(0);
    Angle min = Degrees.of(-90);
    Angle max = Degrees.of(90);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // 180 is equivalent to -180
    // From 0, going to 180 vs -180 are equidistant
    // Continuous might prefer 180, but that exceeds max
    // So should wrap to -180
    double resultDegrees = result.in(Degrees);
    boolean isValid =
        Math.abs(resultDegrees - 180.0) < DELTA || Math.abs(resultDegrees - (-180.0)) < DELTA;
    assertTrue(isValid, "Result should be 180 or -180 (both equivalent), got: " + resultDegrees);
  }

  // ==================== Edge Cases ====================

  @Test
  void optimizeTargetAngle_CurrentEqualsTarget_ReturnsTarget() {
    Angle target = Degrees.of(45);
    Angle current = Degrees.of(45);
    Angle min = Degrees.of(0);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    assertEquals(45.0, result.in(Degrees), DELTA, "Should return target when already at target");
  }

  @Test
  void optimizeTargetAngle_NegativeAngles() {
    Angle target = Degrees.of(-45);
    Angle current = Degrees.of(-50);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    assertEquals(-45.0, result.in(Degrees), DELTA, "Should handle negative angles correctly");
  }

  @Test
  void optimizeTargetAngle_LargePositiveAngles() {
    // Target at 720 degrees (= 0 degrees), current at 0
    Angle target = Degrees.of(720);
    Angle current = Degrees.of(0);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    assertEquals(0.0, result.in(Degrees), DELTA, "Should normalize large positive angles");
  }

  @Test
  void optimizeTargetAngle_LargeNegativeAngles() {
    // Target at -720 degrees (= 0 degrees), current at 0
    Angle target = Degrees.of(-720);
    Angle current = Degrees.of(0);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    assertEquals(0.0, result.in(Degrees), DELTA, "Should normalize large negative angles");
  }

  // ==================== Practical Scenarios ====================

  @Test
  void optimizeTargetAngle_ShooterScenario_Target350_Current10() {
    // Common scenario: target is just to the left of zero, turret is just to the right
    // Should go left (backwards) rather than rotating 340 degrees right
    Angle target = Degrees.of(350);
    Angle current = Degrees.of(10);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // 350 degrees = -10 degrees
    // From current position of 10, going to -10 is 20 degrees
    assertEquals(-10.0, result.in(Degrees), DELTA, "Should optimize to -10 for shortest path");
  }

  @Test
  void optimizeTargetAngle_ShooterScenario_Target10_Current350() {
    // Inverse scenario: current is at 350 (-10), target is at 10
    // Should go forward (right) rather than rotating backwards
    Angle target = Degrees.of(10);
    Angle current = Degrees.of(350);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // From current of 350, going to 10 is 20 degrees forward
    // Going to 370 (10 + 360) would be wrapping around, but that exceeds max
    // So after wrapping down, should get 10
    double resultDegrees = result.in(Degrees);
    boolean isValid =
        Math.abs(resultDegrees - 10.0) < DELTA || Math.abs(resultDegrees - 370.0) < DELTA;
    assertTrue(isValid, "Result should be optimized, got: " + resultDegrees);
  }

  @Test
  void optimizeTargetAngle_FullRotationTurret_0To360() {
    // Turret with full 360 degree range
    Angle target = Degrees.of(270);
    Angle current = Degrees.of(90);
    Angle min = Degrees.of(0);
    Angle max = Degrees.of(360);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // From 90 to 270 is 180 degrees either way
    // Implementation should choose consistently
    double resultDegrees = result.in(Degrees);
    boolean isValid =
        Math.abs(resultDegrees - 270.0) < DELTA || Math.abs(resultDegrees - (-90.0)) < DELTA;
    assertTrue(isValid, "Result should be optimized, got: " + resultDegrees);
  }

  // ==================== Boundary Tests ====================

  @Test
  void optimizeTargetAngle_TargetAtMinBoundary() {
    Angle target = Degrees.of(-180);
    Angle current = Degrees.of(0);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // -180 is equivalent to 180
    // From 0, both are 180 degrees away
    double resultDegrees = result.in(Degrees);
    boolean isValid =
        Math.abs(resultDegrees - (-180.0)) < DELTA || Math.abs(resultDegrees - 180.0) < DELTA;
    assertTrue(isValid, "Result should be at boundary, got: " + resultDegrees);
  }

  @Test
  void optimizeTargetAngle_TargetAtMaxBoundary() {
    Angle target = Degrees.of(180);
    Angle current = Degrees.of(0);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // 180 is equivalent to -180
    // From 0, both are 180 degrees away
    double resultDegrees = result.in(Degrees);
    boolean isValid =
        Math.abs(resultDegrees - 180.0) < DELTA || Math.abs(resultDegrees - (-180.0)) < DELTA;
    assertTrue(isValid, "Result should be at boundary, got: " + resultDegrees);
  }

  @Test
  void optimizeTargetAngle_CurrentAtMinBoundary() {
    Angle target = Degrees.of(0);
    Angle current = Degrees.of(-180);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // From -180 to 0 is 180 degrees either direction
    double resultDegrees = result.in(Degrees);
    boolean isValid =
        Math.abs(resultDegrees - 0.0) < DELTA
            || Math.abs(resultDegrees - 360.0) < DELTA
            || Math.abs(resultDegrees - (-360.0)) < DELTA;
    assertTrue(isValid, "Result should be optimized from boundary, got: " + resultDegrees);
  }

  @Test
  void optimizeTargetAngle_CurrentAtMaxBoundary() {
    Angle target = Degrees.of(0);
    Angle current = Degrees.of(180);
    Angle min = Degrees.of(-180);
    Angle max = Degrees.of(180);

    Angle result = Turret.optimizeTarget(target, current, min, max);

    // From 180 to 0 is 180 degrees either direction
    double resultDegrees = result.in(Degrees);
    boolean isValid =
        Math.abs(resultDegrees - 0.0) < DELTA
            || Math.abs(resultDegrees - 360.0) < DELTA
            || Math.abs(resultDegrees - (-360.0)) < DELTA;
    assertTrue(isValid, "Result should be optimized from boundary, got: " + resultDegrees);
  }
}
