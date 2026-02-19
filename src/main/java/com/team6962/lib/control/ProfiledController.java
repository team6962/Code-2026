package com.team6962.lib.control;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Timer;

/**
 * A controller that follows a motion profile and uses PID feedback to correct for error. This is
 * not recommended for low-level motor control, but is intended to control the motion of high-level
 * systems such as the position of the robot on the field. This is because the output of this
 * controller is a target velocity, not a voltage or current input to a motor, as this does not
 * include the motor feedforward model.
 */
public class ProfiledController {
  /** The motion profile that this controller follows. */
  private MotionProfile profile;

  /** The PID controller used for feedback control to follow the motion profile. */
  private PIDController feedback;

  /** The time when the current motion profile started. */
  private double startTime;

  /** The initial state of the current motion profile. */
  private MotionProfile.State initialState;

  /** The goal state of the current motion profile. */
  private MotionProfile.State goalState;

  /**
   * Creates a new ProfiledController with the given PID gains, motion profile, and update
   * frequency.
   *
   * @param kP The proportional feedback gain.
   * @param kI The integral feedback gain.
   * @param kD The derivative feedback gain.
   * @param profile The motion profile.
   * @param updateFrequency The frequency at which the controller will be updated.
   */
  public ProfiledController(
      double kP, double kI, double kD, MotionProfile profile, Frequency updateFrequency) {
    this.feedback = new PIDController(kP, kI, kD, updateFrequency.asPeriod().in(Seconds));
    this.profile = profile;
  }

  /**
   * Sets a motion profile from the given initial and goal states, using the minimum time required
   * to complete the profile based on the constraints.
   *
   * @param initial The initial state (position and velocity when the motion starts)
   * @param goal The goal state (target position and velocity)
   */
  public void setProfile(MotionProfile.State initialState, MotionProfile.State goalState) {
    this.initialState = initialState;
    this.goalState = goalState;
    profile.setProfile(initialState, goalState);
    startTime = Timer.getFPGATimestamp();
  }

  /**
   * Sets a motion profile from the given initial and goal states, and stretches the profile to take
   * the specified duration.
   *
   * @param initial The initial state (position and velocity when the motion starts)
   * @param goal The goal state (target position and velocity)
   * @param duration The desired duration of the profile in seconds
   */
  public void setProfile(
      MotionProfile.State initialState, MotionProfile.State goalState, double duration) {
    this.initialState = initialState;
    this.goalState = goalState;
    profile.setProfile(initialState, goalState, duration);
    startTime = Timer.getFPGATimestamp();
  }

  /**
   * Sets the duration of the current motion profile. This can be used to stretch the profile to
   * make it take longer than the minimum time required by the physical constraints of the system.
   *
   * @param duration The desired duration of the profile in seconds
   */
  public void setDuration(double duration) {
    profile.setDuration(duration);
  }

  /**
   * Gets the duration of the current motion profile, in seconds.
   *
   * @return The duration of the profile in seconds
   */
  public double getDuration() {
    return profile.getDuration();
  }

  /**
   * Calculates the current state of the motion profile based on the time since the profile started.
   *
   * @return The state of the profile at the current time
   */
  public MotionProfile.State sample() {
    return profile.sampleAt(Timer.getFPGATimestamp() - startTime);
  }

  /**
   * Gets the underlying motion profile.
   *
   * @return The motion profile
   */
  public MotionProfile getProfile() {
    return profile;
  }

  /**
   * Gets the PID controller used for feedback control to follow the motion profile.
   *
   * @return The PID controller
   */
  public PIDController getFeedback() {
    return feedback;
  }

  /**
   * Gets the error between the current position of the system and the current target position of
   * the profile.
   *
   * @param currentPosition The current position of the system
   * @return The error between the current position and the target position
   */
  public double getError(double currentPosition) {
    return sample().position - currentPosition;
  }

  /**
   * Gets the remaining time until the profile is complete, in seconds.
   *
   * @return The remaining time in seconds
   */
  public double getRemainingTime() {
    double time = Timer.getFPGATimestamp() - startTime;
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

  /**
   * Calculates the desired velocity at the current time, given the current state of the system.
   *
   * @param currentState The current state of the system
   * @return The desired velocity that will cause the system to follow the motion profile
   */
  public double calculate(MotionProfile.State currentState) {
    double realTime = Timer.getFPGATimestamp() - startTime;

    if (realTime > getDuration()) {
      if (goalState.velocity != 0) {
        return goalState.velocity;
      } else {
        return feedback.calculate(currentState.position, goalState.position);
      }
    } else {
      MotionProfile.State profileState = profile.sampleAt(realTime);

      double feedbackOutput = feedback.calculate(currentState.position, profileState.position);
      double feedforward = profileState.velocity;

      return feedbackOutput + feedforward;
    }
  }

  /**
   * Calculates the desired velocity at the current time, given the current state of the system and
   * the goal state. If the current profile is finished or if the goal state is different from the
   * current profile's goal state, a new profile will be generated from the current state to the new
   * goal state before calculating the desired velocity.
   *
   * @param currentState The current state of the system
   * @param goalState The goal state of the system
   * @return The desired velocity that will cause the system to follow the motion profile
   */
  public double calculate(MotionProfile.State currentState, MotionProfile.State goalState) {
    if (isFinished()
        || Math.abs(goalState.position - currentState.position) > 1E-6
        || Math.abs(goalState.velocity - currentState.velocity) > 1E-6) {
      setProfile(currentState, goalState);
    }

    return calculate(currentState);
  }

  /**
   * Gets the goal state of the current motion profile.
   *
   * @return The goal state of the current motion profile
   */
  public MotionProfile.State getGoalState() {
    return goalState;
  }

  /**
   * Gets the initial state of the current motion profile.
   *
   * @return The initial state of the current motion profile
   */
  public MotionProfile.State getInitialState() {
    return initialState;
  }
}
