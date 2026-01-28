package com.team6962.lib.control;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Timer;

/**
 * A controller that can be used to follow trapezoidal motion profiles with PID feedback. This
 * controller is designed to work at a high level, generating velocity setpoints to be sent to a
 * lower-level velocity controller.
 *
 * <h3>Usage</h3>
 *
 * To use this controller, first create an instance of the {@link TrapezoidalController} class,
 * providing the PID constants, motion profile constraints (maximum velocity and acceleration), and
 * update frequency. Next, set up a motion profile by calling the {@link #setProfile setProfile()}
 * method with the initial and goal states. The {@link #calculate calculate()} method can then be
 * called periodically to get the desired velocity that will move the system along the profile.
 *
 * <p>The {@link #getDuration()} method can be used to get the expected total time the profile will
 * take to complete (the time it will take the system to reach the goal state from the initial
 * state). The {@link #getRemainingTime()} method can be used to get the estimated time until the
 * profile will be completed. The {@link #isFinished()} method can be used to check if the profile
 * has been completed.
 *
 * <p>Additionally, the controller supports stretching the profile to make it take longer than the
 * minimum time required by the physical constraints of the system. This can be done by calling the
 * {@link #setDuration setDuration()} method with the desired duration.
 */
public class TrapezoidalController {
  /** The trapezoidal motion profile used to generate setpoints. */
  private final TrapezoidProfile profile;

  /** The PID controller used for feedback control. */
  private final PIDController feedback;

  /** The initial time when the profile started, in seconds since the FPGA started. */
  private double initialTime;

  /** The initial and goal states of the motion profile. */
  private TrapezoidProfile.State initialState, goalState;

  /**
   * A scaling factor for the time duration of the profile. This value should always be greater than
   * or equal to 1, making the profile take longer than or equal to the minimum time required by the
   * physical constraints of the system.
   */
  private double timeScale = 1.0;

  public TrapezoidalController(
      double kP,
      double kI,
      double kD,
      TrapezoidProfile.Constraints constraints,
      Frequency updateFrequency) {
    this.profile = new TrapezoidProfile(constraints);
    this.feedback = new PIDController(kP, kI, kD, updateFrequency.asPeriod().in(Seconds));
  }

  /**
   * Sets up a new motion profile from the given initial and goal states.
   *
   * @param initial The initial state (position and velocity when the motion starts)
   * @param goal The goal state (target position and velocity)
   */
  public void setProfile(State initial, State goal) {
    timeScale = 1.0;
    initialTime = Timer.getFPGATimestamp();

    this.initialState = initial;
    this.goalState = goal;

    profile.calculate(0, initial, goal);
  }

  /**
   * Sets up a new motion profile from the given initial and goal states, and stretches the profile
   * to take the specified duration.
   *
   * @param initial The initial state (position and velocity when the motion starts)
   * @param goal The goal state (target position and velocity)
   * @param duration The desired duration of the profile in seconds
   */
  public void setProfile(State initial, State goal, double duration) {
    setProfile(initial, goal);
    setDuration(duration);
  }

  /**
   * Gets the duration of the current motion profile, in seconds.
   *
   * @return The duration of the profile in seconds
   */
  public double getDuration() {
    return profile.totalTime() * timeScale;
  }

  /**
   * Stretches the current motion profile to take the specified duration.
   *
   * @param duration The desired duration of the profile in seconds
   */
  public void setDuration(double duration) {
    double unscaledDuration = profile.totalTime();

    if (unscaledDuration == 0) {
      timeScale = 1.0;
      return;
    }

    timeScale = duration / unscaledDuration;
  }

  /**
   * Samples the motion profile at the given time, adjusting for the time scaling.
   *
   * @param time The time at which to sample the profile, in seconds
   * @return The state of the profile at the given time
   */
  private TrapezoidProfile.State sampleAt(double time) {
    TrapezoidProfile.State originalState =
        profile.calculate(time / timeScale, initialState, goalState);

    return new TrapezoidProfile.State(originalState.position, originalState.velocity / timeScale);
  }

  /**
   * Calculates the desired velocity at the current time, given the current state of the system.
   *
   * @param current The current state of the system (position and velocity)
   * @return The desired velocity that will cause the system to follow the motion profile
   */
  public double calculate(State current) {
    double time = (Timer.getFPGATimestamp() - initialTime) / timeScale;

    if (time > getDuration()) {
      if (goalState.velocity != 0) {
        return goalState.velocity;
      } else {
        return feedback.calculate(current.position, goalState.position);
      }
    } else {
      State profileState = sampleAt(time);

      double feedbackOutput = feedback.calculate(current.position, profileState.position);
      double feedforward = profileState.velocity;

      return feedbackOutput + feedforward;
    }
  }

  /**
   * Gets the underlying trapezoidal motion profile. Note that the profile is not stretched, so the
   * time scaling must be applied separately.
   *
   * @return The trapezoidal motion profile
   */
  public TrapezoidProfile getProfile() {
    return profile;
  }

  /**
   * Gets the PID controller used for feedback control.
   *
   * @return The PID controller
   */
  public PIDController getFeedback() {
    return feedback;
  }

  /**
   * Gets the remaining time until the profile is complete, in seconds.
   *
   * @return The remaining time in seconds
   */
  public double getRemainingTime() {
    double time = (Timer.getFPGATimestamp() - initialTime) * timeScale;
    return Math.max(0, getDuration() - time);
  }

  /**
   * Checks if the profile has finished.
   *
   * @return True if the profile is finished, false otherwise
   */
  public boolean isFinished() {
    return getRemainingTime() <= 0;
  }
}
