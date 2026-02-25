package com.team6962.lib.control;

import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Represents a motion profile that can be sampled at any point in time to get the desired position
 * and velocity of the system, or followed with a {@link ProfiledController}.
 */
public interface MotionProfile {
  /**
   * Sets a motion profile from the given initial and goal states, using the minimum time required
   * to complete the profile based on the constraints.
   *
   * @param initial The initial state (position and velocity when the motion starts)
   * @param goal The goal state (target position and velocity)
   */
  public void setProfile(State initial, State goal);

  /**
   * Sets a motion profile from the given initial and goal states, and stretches the profile to take
   * the specified duration.
   *
   * @param initial The initial state (position and velocity when the motion starts)
   * @param goal The goal state (target position and velocity)
   * @param duration The desired duration of the profile in seconds
   */
  public default void setProfile(State initial, State goal, double duration) {
    setProfile(initial, goal);
    setDuration(duration);
  }

  /**
   * Sets the duration of the current motion profile. This can be used to stretch the profile to
   * make it take longer than the minimum time required by the physical constraints of the system.
   *
   * @param duration The desired duration of the profile in seconds
   */
  public void setDuration(double duration);

  /**
   * Gets the duration of the current motion profile, in seconds.
   *
   * @return The duration of the profile in seconds
   */
  public double getDuration();

  /**
   * Gets the state of the motion profile at the given time.
   *
   * @param time The time since the profile started, in seconds
   * @return The state of the profile at the given time
   */
  public State sampleAt(double time);

  /**
   * A class representing the state of a motion profile at a given time, with a position and
   * velocity. This is used for both the initial and goal states of a profile, as well as the state
   * at any point in time when sampling the profile.
   */
  public static class State {
    public double position;
    public double velocity;

    public State(double position, double velocity) {
      this.position = position;
      this.velocity = velocity;
    }

    public State(TrapezoidProfile.State state) {
      this.position = state.position;
      this.velocity = state.velocity;
    }

    public State(ExponentialProfile.State state) {
      this.position = state.position;
      this.velocity = state.velocity;
    }

    public TrapezoidProfile.State toTrapezoidal() {
      return new TrapezoidProfile.State(position, velocity);
    }

    public ExponentialProfile.State toExponential() {
      return new ExponentialProfile.State(position, velocity);
    }

    public State extrapolate(double time) {
      return new State(position + velocity * time, velocity);
    }
  }
}
