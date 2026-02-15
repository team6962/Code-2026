package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.Test;

/**
 * Property-based tests for {@link SwerveKinematicsUtil#optimize}.
 *
 * <p><b>Why this test exists (for students):</b> The swerve module optimization is a great example
 * of code that "seems to work" when you test it manually with a few angles, but has subtle
 * properties that are easy to break during refactoring. These tests don't check specific numbers —
 * they check <i>properties</i> that must always hold, no matter what inputs you give:
 *
 * <ol>
 *   <li>The optimized state must produce the same wheel movement as the original.
 *   <li>The optimized angle must be within ±90° of the current wheel angle.
 *   <li>The speed must not change in magnitude (only sign may flip).
 * </ol>
 *
 * <p>If any of these properties break, the robot's swerve drive will behave incorrectly — wheels
 * could fight each other, drive the wrong direction, or oscillate. These tests catch those bugs
 * before they reach the field.
 */
class SwerveOptimizePropertyTest {

  private static final double DELTA = 1e-6;

  /**
   * The key insight: optimizing should NOT change the physical movement of the wheel. If the
   * original state says "go 1 m/s at 135°" and the optimized state says "go -1 m/s at -45°", those
   * produce exactly the same robot movement.
   *
   * <p>We verify this by computing the X and Y velocity components of both states.
   */
  private void assertSameMovement(
      SwerveModuleState original, SwerveModuleState optimized, String context) {
    double origVx = original.speedMetersPerSecond * original.angle.getCos();
    double origVy = original.speedMetersPerSecond * original.angle.getSin();
    double optVx = optimized.speedMetersPerSecond * optimized.angle.getCos();
    double optVy = optimized.speedMetersPerSecond * optimized.angle.getSin();

    assertEquals(
        origVx, optVx, 0.001, context + ": X velocity component changed after optimization");
    assertEquals(
        origVy, optVy, 0.001, context + ": Y velocity component changed after optimization");
  }

  /**
   * The optimized angle must be within ±90° of the current steer angle. This ensures the wheel
   * never has to rotate more than a quarter turn to reach its target.
   */
  private void assertWithin90Degrees(Angle steerAngle, SwerveModuleState optimized, String ctx) {
    double diff =
        Math.abs(
            AngleMath.toDiscrete(optimized.angle.getMeasure().minus(steerAngle)).in(Rotations));
    assertTrue(
        diff <= 0.25 + DELTA,
        ctx
            + ": optimized angle should be within ±90° of steer angle, but diff is "
            + (diff * 360)
            + "°");
  }

  /** Speed magnitude should be preserved (only sign may flip). */
  private void assertSameSpeedMagnitude(
      SwerveModuleState original, SwerveModuleState optimized, String ctx) {
    assertEquals(
        Math.abs(original.speedMetersPerSecond),
        Math.abs(optimized.speedMetersPerSecond),
        DELTA,
        ctx + ": speed magnitude changed after optimization");
  }

  // ==================== Property checks across many angle combinations ====================

  @Test
  void optimize_AllProperties_HoldForManyAngles() {
    double[] targetAngles = {0, 30, 45, 60, 90, 120, 135, 150, 180, -30, -45, -90, -135, -180};
    double[] steerAngles = {0, 45, 90, 135, 180, -45, -90, -135, -180, 270, -270};
    double[] speeds = {1.0, 3.5, -2.0};

    for (double targetDeg : targetAngles) {
      for (double steerDeg : steerAngles) {
        for (double speed : speeds) {
          SwerveModuleState original =
              new SwerveModuleState(speed, Rotation2d.fromDegrees(targetDeg));
          Angle steer = Degrees.of(steerDeg);
          SwerveModuleState optimized = SwerveKinematicsUtil.optimize(original, steer);

          String ctx =
              String.format("target=%.0f°, steer=%.0f°, speed=%.1f", targetDeg, steerDeg, speed);

          assertSameMovement(original, optimized, ctx);
          assertWithin90Degrees(steer, optimized, ctx);
          assertSameSpeedMagnitude(original, optimized, ctx);
        }
      }
    }
  }

  // ==================== Individual illustrative examples ====================

  @Test
  void optimize_WheelAt0_TargetAt135_InvertsAndFlipsTo_Neg45() {
    // Wheel is pointing at 0°. Target is 135° at 2 m/s.
    // 135° is > 90° away, so it's faster to flip: drive at -2 m/s toward -45°.
    // -45° is only 45° from the current 0° position.
    SwerveModuleState original = new SwerveModuleState(2.0, Rotation2d.fromDegrees(135));
    SwerveModuleState optimized = SwerveKinematicsUtil.optimize(original, Degrees.of(0));

    assertEquals(-2.0, optimized.speedMetersPerSecond, DELTA, "Speed should be negated");
    assertEquals(-45.0, optimized.angle.getDegrees(), 0.01, "Angle should flip to -45°");
    assertSameMovement(original, optimized, "135° → -45° flip");
  }

  @Test
  void optimize_ZeroSpeed_AngleStillOptimized() {
    // Even when speed is 0, the angle should still be optimized so the wheel
    // is ready to drive in the right direction quickly.
    SwerveModuleState original = new SwerveModuleState(0.0, Rotation2d.fromDegrees(180));
    SwerveModuleState optimized = SwerveKinematicsUtil.optimize(original, Degrees.of(0));

    assertWithin90Degrees(Degrees.of(0), optimized, "Zero speed");
  }

  @Test
  void optimize_NegativeSpeed_HandledCorrectly() {
    // If the input speed is already negative, optimization should still work.
    SwerveModuleState original = new SwerveModuleState(-3.0, Rotation2d.fromDegrees(0));
    SwerveModuleState optimized = SwerveKinematicsUtil.optimize(original, Degrees.of(170));

    assertSameMovement(original, optimized, "Negative input speed");
    assertWithin90Degrees(Degrees.of(170), optimized, "Negative input speed");
  }

  @Test
  void optimize_IdempotentWhenAligned() {
    // When the target is already within 90° of the steer angle, optimization
    // should not change anything.
    SwerveModuleState original = new SwerveModuleState(2.0, Rotation2d.fromDegrees(30));
    Angle steer = Degrees.of(0);
    SwerveModuleState optimized = SwerveKinematicsUtil.optimize(original, steer);

    assertEquals(2.0, optimized.speedMetersPerSecond, DELTA, "Speed should not change");
    assertEquals(
        30.0, optimized.angle.getDegrees(), 0.01, "Angle should not change when within 90°");
  }
}
