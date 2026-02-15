package com.team6962.lib.control;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.junit.jupiter.api.Assertions.*;

import com.team6962.lib.math.TranslationalVelocity;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link TranslationController} — zero and near-zero length paths.
 *
 * <p>Previously, {@code getPathDirectionVector()} called {@code .unit()} on a zero-length vector,
 * producing NaN that corrupted the rotation matrix and triggered a {@code SingularMatrixException}.
 * Now fixed: paths shorter than 1µm return an arbitrary unit vector.
 */
class TranslationControllerTest {

  private TranslationController controller;

  @BeforeEach
  void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    SimHooks.stepTiming(0);

    controller =
        new TranslationController(
            1.0, 0, 0, new TrapezoidProfile.Constraints(1.0, 1.0), Hertz.of(50));
  }

  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
  }

  private static double vx(TranslationalVelocity v) {
    return v.x.in(MetersPerSecond);
  }

  private static double vy(TranslationalVelocity v) {
    return v.y.in(MetersPerSecond);
  }

  // ==================== Exact zero: start == goal ====================

  /**
   * When start == goal with zero velocity, the controller should be a no-op: already at the goal,
   * output zero velocity.
   *
   * <p>Previously crashed with SingularMatrixException.
   */
  @Test
  void calculate_StartEqualsGoal_ReturnsZeroVelocity() {
    Translation2d samePoint = new Translation2d(1, 1);

    controller.setProfile(
        samePoint, TranslationalVelocity.ZERO, samePoint, TranslationalVelocity.ZERO);

    SimHooks.stepTiming(0.02);

    TranslationalVelocity output = controller.calculate(samePoint, TranslationalVelocity.ZERO);

    assertEquals(0.0, vx(output), 0.01, "At goal: X velocity should be ~0");
    assertEquals(0.0, vy(output), 0.01, "At goal: Y velocity should be ~0");
  }

  /**
   * Same bug with nonzero initial velocity — robot commanded to hold position while moving.
   *
   * <p>Previously crashed with SingularMatrixException.
   */
  @Test
  void setProfile_StartEqualsGoalWithVelocity_DoesNotCrash() {
    Translation2d samePoint = new Translation2d(1, 1);
    TranslationalVelocity someVelocity = new TranslationalVelocity(0.5, 0.5);

    assertDoesNotThrow(
        () ->
            controller.setProfile(samePoint, someVelocity, samePoint, TranslationalVelocity.ZERO));

    double duration = controller.getDuration();
    assertFalse(Double.isNaN(duration), "Duration should not be NaN");
    assertTrue(duration >= 0, "Duration should be non-negative");
  }

  // ==================== Near-zero: floating-point-noise distances ====================

  /**
   * The {@code norm == 0} guard is insufficient. Two computations of “the same point” can differ by
   * floating-point noise (~2e-16 m), producing a nonzero norm that passes exact-zero. The resulting
   * path direction is pure noise.
   *
   * <p>Fix: use {@code norm < 1e-6} (1 µm) so sub-precision distances are treated as zero.
   */
  @Test
  void calculate_NearZeroPath_FloatingPointNoise_DirectionIsArbitrary() {
    Translation2d start =
        new Translation2d(0.1 + 0.2, Math.sqrt(2.0) * Math.sqrt(0.5)); // (0.300...04, 1.000...02)
    Translation2d goal = new Translation2d(0.3, 1.0);

    // Sanity: these are NOT bitwise equal
    assertNotEquals(start.getX(), goal.getX(), "Sanity: X coords should differ by FP noise");
    assertNotEquals(start.getY(), goal.getY(), "Sanity: Y coords should differ by FP noise");

    double distance = start.getDistance(goal);
    assertTrue(distance < 1e-14, "Distance should be at FP noise level, got: " + distance);
    assertTrue(distance > 0, "Distance should be nonzero (passes the == 0 check)");

    controller.setProfile(start, TranslationalVelocity.ZERO, goal, TranslationalVelocity.ZERO);

    SimHooks.stepTiming(0.02);

    TranslationalVelocity output = controller.calculate(start, TranslationalVelocity.ZERO);

    // With the epsilon threshold fix, this is treated as a zero-distance path.
    // Before the fix, the output had a noise-determined direction.
    assertFalse(Double.isNaN(vx(output)), "Output should not be NaN");
    assertFalse(Double.isNaN(vy(output)), "Output should not be NaN");
  }

  /**
   * Near-zero path with velocity: noise-direction coordinate transform can misroute deceleration
   * into the wrong axis.
   */
  @Test
  void setProfile_NearZeroPathWithVelocity_DirectionUnreliable() {
    Translation2d start = new Translation2d(1.0 / 3.0 * 3.0, 1.0); // (1.0, 1.0) via roundabout
    Translation2d goal = new Translation2d(1.0, 1.0);

    TranslationalVelocity moving = new TranslationalVelocity(1.0, 0.0);

    assertDoesNotThrow(
        () -> controller.setProfile(start, moving, goal, TranslationalVelocity.ZERO),
        "Should not crash for near-zero distance path");

    double duration = controller.getDuration();
    assertFalse(Double.isNaN(duration), "Duration should not be NaN");
    assertFalse(Double.isInfinite(duration), "Duration should not be infinite");
  }
}
