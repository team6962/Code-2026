package com.team6962.lib.control;

import static edu.wpi.first.units.Units.Hertz;
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
 * Tests for {@link TranslationController} — bug exposure.
 *
 * <p>Demonstrates the crash when start == goal: {@code getPathDirectionVector()} calls {@code
 * .unit()} on a zero-length vector, producing NaN that corrupts the rotation matrix and triggers a
 * {@code SingularMatrixException} inside {@code getFieldToPathMatrix().inv()}.
 *
 * <p>This is a real scenario: commanding the robot to hold its current position while the
 * TranslationController is active.
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

  /**
   * BUG: when initialPosition == goalPosition with zero velocity, setProfile throws
   * SingularMatrixException. A well-behaved controller should return zero velocity.
   */
  @Test
  void calculate_StartEqualsGoal_BUG_ThrowsSingularMatrixException() {
    Translation2d samePoint = new Translation2d(1, 1);

    assertThrows(
        org.ejml.data.SingularMatrixException.class,
        () ->
            controller.setProfile(
                samePoint, TranslationalVelocity.ZERO, samePoint, TranslationalVelocity.ZERO),
        "BUG: setProfile throws SingularMatrixException when start == goal");
  }

  /**
   * Same bug with nonzero initial velocity — the robot is moving and is commanded to hold its
   * current position. The NaN rotation matrix makes setProfile crash.
   */
  @Test
  void setProfile_StartEqualsGoalWithVelocity_BUG_ThrowsSingularMatrixException() {
    Translation2d samePoint = new Translation2d(1, 1);
    TranslationalVelocity someVelocity = new TranslationalVelocity(0.5, 0.5);

    assertThrows(
        org.ejml.data.SingularMatrixException.class,
        () -> controller.setProfile(samePoint, someVelocity, samePoint, TranslationalVelocity.ZERO),
        "BUG: setProfile throws SingularMatrixException when start == goal with velocity");
  }
}
