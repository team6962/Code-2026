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
 * Tests for {@link TranslationController} — start == goal (zero-length path).
 *
 * <p>Previously, {@code getPathDirectionVector()} called {@code .unit()} on a zero-length vector,
 * producing NaN that corrupted the rotation matrix and triggered a {@code SingularMatrixException}.
 * Now fixed: a zero-length path returns an arbitrary unit vector.
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

  /**
   * When start == goal with zero velocity, the controller should be a no-op: already at the goal,
   * output zero velocity.
   *
   * <p>Previously crashed with SingularMatrixException.
   */
  @Test
  void calculate_StartEqualsGoal_ReturnsZeroVelocity() {
    Translation2d samePoint = new Translation2d(1, 1);

    // Should not throw
    controller.setProfile(
        samePoint, TranslationalVelocity.ZERO, samePoint, TranslationalVelocity.ZERO);

    SimHooks.stepTiming(0.02);

    TranslationalVelocity output = controller.calculate(samePoint, TranslationalVelocity.ZERO);

    assertEquals(0.0, vx(output), 0.01, "At goal: X velocity should be ~0");
    assertEquals(0.0, vy(output), 0.01, "At goal: Y velocity should be ~0");
  }

  /**
   * When start == goal with nonzero initial velocity, the controller should decelerate to zero.
   * This can happen when a robot is commanded to hold its current position while moving.
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
}
