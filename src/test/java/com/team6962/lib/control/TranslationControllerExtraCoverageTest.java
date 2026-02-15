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
 * Broader coverage for {@link TranslationController}: profile setup, axis motion, finish detection,
 * and duration scaling.
 *
 * <p>Uses SimHooks to control the FPGA timer deterministically.
 */
class TranslationControllerExtraCoverageTest {

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

  // ==================== Profile setup ====================

  @Test
  void setProfile_DifferentPoints_HasPositiveDuration() {
    controller.setProfile(
        new Translation2d(0, 0),
        TranslationalVelocity.ZERO,
        new Translation2d(1, 0),
        TranslationalVelocity.ZERO);

    assertTrue(controller.getDuration() > 0, "Moving 1m should have positive duration");
    assertFalse(controller.isFinished(), "Should not be finished immediately after setProfile");
  }

  @Test
  void setProfile_DiagonalPath_HasPositiveDuration() {
    controller.setProfile(
        new Translation2d(0, 0),
        TranslationalVelocity.ZERO,
        new Translation2d(1, 1),
        TranslationalVelocity.ZERO);

    double duration = controller.getDuration();
    assertTrue(duration > 0, "Diagonal path should have positive duration");
    assertTrue(
        duration > Math.sqrt(2),
        "Duration should be longer than distance/maxVelocity due to accel/decel");
  }

  // ==================== isFinished ====================

  @Test
  void isFinished_AfterDuration_ReturnsTrue() {
    controller.setProfile(
        new Translation2d(0, 0),
        TranslationalVelocity.ZERO,
        new Translation2d(1, 0),
        TranslationalVelocity.ZERO);

    double duration = controller.getDuration();
    SimHooks.stepTiming(duration + 0.5);

    assertTrue(controller.isFinished(), "Should be finished after profile duration elapses");
  }

  // ==================== Axis motion ====================

  @Test
  void calculate_MovingAlongX_ProducesPositiveXVelocity() {
    controller.setProfile(
        new Translation2d(0, 0),
        TranslationalVelocity.ZERO,
        new Translation2d(2, 0),
        TranslationalVelocity.ZERO);

    SimHooks.stepTiming(0.02);

    TranslationalVelocity output =
        controller.calculate(new Translation2d(0, 0), TranslationalVelocity.ZERO);

    assertTrue(vx(output) > 0, "Should produce positive X velocity, got: " + vx(output));
    assertEquals(0.0, vy(output), 0.01, "Purely X motion should have ~zero Y velocity");
  }

  @Test
  void calculate_MovingAlongY_ProducesPositiveYVelocity() {
    controller.setProfile(
        new Translation2d(0, 0),
        TranslationalVelocity.ZERO,
        new Translation2d(0, 2),
        TranslationalVelocity.ZERO);

    SimHooks.stepTiming(0.02);

    TranslationalVelocity output =
        controller.calculate(new Translation2d(0, 0), TranslationalVelocity.ZERO);

    assertEquals(0.0, vx(output), 0.01, "Purely Y motion should have ~zero X velocity");
    assertTrue(vy(output) > 0, "Should produce positive Y velocity, got: " + vy(output));
  }

  @Test
  void calculate_DiagonalPath_ProducesVelocityInCorrectDirection() {
    controller.setProfile(
        new Translation2d(0, 0),
        TranslationalVelocity.ZERO,
        new Translation2d(1, 1),
        TranslationalVelocity.ZERO);

    SimHooks.stepTiming(0.02);

    TranslationalVelocity output =
        controller.calculate(new Translation2d(0, 0), TranslationalVelocity.ZERO);

    assertTrue(vx(output) > 0, "X velocity should be positive for diagonal path");
    assertTrue(vy(output) > 0, "Y velocity should be positive for diagonal path");
    assertEquals(
        vx(output),
        vy(output),
        0.01,
        "X and Y velocities should be roughly equal for a 45-degree path");
  }

  // ==================== setDuration ====================

  @Test
  void setDuration_SynchronizesBothAxes() {
    controller.setProfile(
        new Translation2d(0, 0),
        TranslationalVelocity.ZERO,
        new Translation2d(1, 0),
        TranslationalVelocity.ZERO);

    double originalDuration = controller.getDuration();
    double newDuration = originalDuration * 2;
    controller.setDuration(newDuration);

    assertEquals(
        newDuration,
        controller.getDuration(),
        0.01,
        "getDuration should return the new stretched duration");
  }
}
