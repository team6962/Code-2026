package com.team6962.lib.control;

import static edu.wpi.first.units.Units.Hertz;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link TrapezoidalController}.
 *
 * <p>These tests use WPILib's SimHooks to pause and step the FPGA timer, giving us deterministic
 * control over the passage of time.
 */
class TrapezoidalControllerTest {

  private static final double DELTA = 1e-6;

  @BeforeEach
  void setUp() {
    // Initialize HAL so Timer.getFPGATimestamp() works in tests
    HAL.initialize(500, 0);
    // Pause time so it only advances when we explicitly step it
    SimHooks.pauseTiming();
    // Reset to a known starting point
    SimHooks.stepTiming(0);
  }

  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
  }

  /** Helper: create a controller with reasonable defaults. */
  private TrapezoidalController makeController(double kP) {
    // Max velocity 1 m/s, max acceleration 1 m/s^2
    return new TrapezoidalController(
        kP, 0, 0, new TrapezoidProfile.Constraints(1.0, 1.0), Hertz.of(50));
  }

  // ==================== getDuration ====================

  @Test
  void getDuration_SimpleProfile_ReturnsPositive() {
    TrapezoidalController ctrl = makeController(0);
    ctrl.setProfile(
        new TrapezoidProfile.State(0, 0), // start at 0, stationary
        new TrapezoidProfile.State(1, 0) // move to 1, stop
        );

    double duration = ctrl.getDuration();

    // With max vel=1, max accel=1, moving 1 meter:
    // accel for 1s to reach v=1, then decel for 1s => 2 seconds total
    assertTrue(duration > 0, "Duration should be positive");
    assertEquals(2.0, duration, 0.01, "Should take about 2s to move 1m with v_max=1, a_max=1");
  }

  @Test
  void getDuration_ZeroDistanceProfile_ReturnsZero() {
    TrapezoidalController ctrl = makeController(1);
    ctrl.setProfile(new TrapezoidProfile.State(5, 0), new TrapezoidProfile.State(5, 0));

    assertEquals(0.0, ctrl.getDuration(), DELTA, "Zero-distance profile should have zero duration");
  }

  // ==================== setDuration (stretching) ====================

  @Test
  void setDuration_StretchesProfile() {
    TrapezoidalController ctrl = makeController(0);
    ctrl.setProfile(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(1, 0));

    double originalDuration = ctrl.getDuration();
    ctrl.setDuration(originalDuration * 2);

    assertEquals(
        originalDuration * 2,
        ctrl.getDuration(),
        DELTA,
        "Duration should be doubled after setDuration");
  }

  @Test
  void setDuration_ZeroDurationProfile_DoesNotCrash() {
    TrapezoidalController ctrl = makeController(1);
    ctrl.setProfile(new TrapezoidProfile.State(5, 0), new TrapezoidProfile.State(5, 0));

    // Should not throw or produce NaN/Infinity
    assertDoesNotThrow(() -> ctrl.setDuration(3.0));
    assertFalse(Double.isNaN(ctrl.getDuration()));
    assertFalse(Double.isInfinite(ctrl.getDuration()));
  }

  // ==================== isFinished / getRemainingTime ====================

  @Test
  void isFinished_BeforeProfileCompletes_ReturnsFalse() {
    TrapezoidalController ctrl = makeController(0);
    ctrl.setProfile(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(1, 0));

    double duration = ctrl.getDuration();

    // Step to halfway through the profile
    SimHooks.stepTiming(duration / 2);

    assertFalse(ctrl.isFinished(), "Should NOT be finished halfway through");
    assertTrue(
        ctrl.getRemainingTime() > 0,
        "Remaining time should be positive halfway through, got: " + ctrl.getRemainingTime());
  }

  @Test
  void isFinished_AfterProfileCompletes_ReturnsTrue() {
    TrapezoidalController ctrl = makeController(0);
    ctrl.setProfile(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(1, 0));

    double duration = ctrl.getDuration();

    // Step past the end of the profile
    SimHooks.stepTiming(duration + 0.1);

    assertTrue(ctrl.isFinished(), "Should be finished after profile duration");
    assertEquals(
        0.0, ctrl.getRemainingTime(), DELTA, "Remaining time should be zero when finished");
  }

  // ==================== BUG: getRemainingTime with timeScale ====================

  /**
   * Verifies that getRemainingTime() and isFinished() work correctly with stretched profiles.
   *
   * <p>Previously, getRemainingTime() used (elapsed * timeScale) instead of just (elapsed), which
   * double-scaled the time and caused isFinished() to return true at the midpoint.
   */
  @Test
  void getRemainingTime_WithStretchedProfile_WorksCorrectly() {
    TrapezoidalController ctrl = makeController(0);
    ctrl.setProfile(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(1, 0));

    double naturalDuration = ctrl.getDuration(); // ~2s
    double stretchedDuration = naturalDuration * 2; // 4s
    ctrl.setDuration(stretchedDuration);

    assertEquals(
        stretchedDuration,
        ctrl.getDuration(),
        DELTA,
        "Sanity check: getDuration should return the stretched duration");

    // Step to the halfway point of the stretched profile
    SimHooks.stepTiming(stretchedDuration / 2);

    assertEquals(
        stretchedDuration / 2,
        ctrl.getRemainingTime(),
        0.01,
        "Remaining time should be half the stretched duration at the midpoint");
    assertFalse(ctrl.isFinished(), "Should NOT be finished at the midpoint of a stretched profile");
  }

  // ==================== BUG: calculate() time comparison mismatch ====================

  /**
   * Verifies that calculate() correctly enters the post-profile branch for stretched profiles.
   *
   * <p>Previously, calculate() compared profile-time against real-time getDuration(), so the
   * post-profile branch was unreachable for timeScale > 1.
   */
  @Test
  void calculate_WithStretchedProfile_EntersPostProfileBranch() {
    TrapezoidalController ctrl = makeController(1.0);
    ctrl.setProfile(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(1, 0));

    double naturalDuration = ctrl.getDuration(); // ~2s
    ctrl.setDuration(naturalDuration * 2); // stretch to ~4s
    double stretchedDuration = ctrl.getDuration();

    // Step well past the stretched profile's real-time end
    SimHooks.stepTiming(stretchedDuration + 2.0);

    // After profile ends with goalVelocity=0, on-target output should be ~0
    double output = ctrl.calculate(new TrapezoidProfile.State(1, 0));
    assertEquals(0.0, output, 0.01, "At goal after stretched profile: output should be ~0");

    // Off-target: post-profile branch uses pure PID to hold at goal position
    // PID(0.5, 1.0) with kP=1.0 → 0.5
    double offTargetOutput = ctrl.calculate(new TrapezoidProfile.State(0.5, 0));
    assertEquals(
        0.5, offTargetOutput, 0.01, "Off-target after profile: should get pure PID feedback");
  }

  /**
   * Verifies that after a stretched profile with nonzero goal velocity, calculate() returns the
   * goal velocity (the post-profile feedforward branch).
   *
   * <p>Previously, the in-profile branch ran instead because of a time-unit mismatch.
   */
  @Test
  void calculate_StretchedWithGoalVelocity_ReturnsGoalVelocity() {
    TrapezoidalController ctrl = makeController(1.0);
    // Move from 0 to 1, arriving with velocity 0.5
    ctrl.setProfile(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(1, 0.5));

    double naturalDuration = ctrl.getDuration();
    assertTrue(naturalDuration > 0, "Sanity: profile should have nonzero duration");

    ctrl.setDuration(naturalDuration * 2);
    double stretchedDuration = ctrl.getDuration();

    // Step just past the stretched duration
    SimHooks.stepTiming(stretchedDuration + 0.5);

    // After profile with goalVelocity=0.5, at the goal position:
    // the post-profile branch should return exactly the goal velocity.
    double output = ctrl.calculate(new TrapezoidProfile.State(1, 0.5));
    assertEquals(0.5, output, DELTA, "Should return goal velocity after stretched profile ends");
  }

  // ==================== calculate: basic correctness ====================

  @Test
  void calculate_AtStart_OutputsNonzeroVelocity() {
    TrapezoidalController ctrl = makeController(1.0);
    ctrl.setProfile(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(1, 0));

    // At t=0, the profile should start accelerating
    // Tiny step so calculate sees some elapsed time
    SimHooks.stepTiming(0.02);

    double output = ctrl.calculate(new TrapezoidProfile.State(0, 0));

    // Feedforward from the profile should give some positive velocity
    // plus PID feedback since we're behind the profile
    assertTrue(output > 0, "Should output positive velocity to start moving toward goal");
  }

  @Test
  void calculate_NoStretch_FinishesCorrectly() {
    TrapezoidalController ctrl = makeController(0);
    ctrl.setProfile(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(1, 0));

    double duration = ctrl.getDuration();

    // Step past the profile
    SimHooks.stepTiming(duration + 0.5);

    // With kP=0, after profile ends with goalVelocity=0, output should be 0
    double output = ctrl.calculate(new TrapezoidProfile.State(1, 0));
    assertEquals(0.0, output, 0.01, "After profile with zero goal velocity, output should be ~0");
  }

  // ==================== setProfile resets state ====================

  @Test
  void setProfile_ResetsTimer() {
    TrapezoidalController ctrl = makeController(0);
    ctrl.setProfile(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(1, 0));

    double duration = ctrl.getDuration();
    SimHooks.stepTiming(duration + 1);
    assertTrue(ctrl.isFinished(), "Should be finished after first profile");

    // Set a new profile — should reset
    ctrl.setProfile(new TrapezoidProfile.State(0, 0), new TrapezoidProfile.State(2, 0));
    assertFalse(ctrl.isFinished(), "Should NOT be finished immediately after setting new profile");
    assertTrue(ctrl.getDuration() > 0, "New profile should have positive duration");
  }
}
