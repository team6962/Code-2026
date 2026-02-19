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
  /** The constraints for the trapezoidal motion profile. */
  private final TrapezoidProfile.Constraints constraints;

  /** The trapezoidal motion profile used to generate setpoints. */
  private TrapezoidProfile profile;

  /** The PID controller used for feedback control. */
  private PIDController feedback;

  /** The initial time when the profile started, in seconds since the FPGA started. */
  private double initialTime;

  /** The initial and goal states of the motion profile. */
  private TrapezoidProfile.State initialState, goalState;

  /**
   * Creates a new TrapezoidalController with the specified PID gains, constraints, and update
   * frequency.
   * 
   * @param kP The proportional gain
   * @param kI The integral gain
   * @param kD The derivative gain
   * @param constraints The motion profile constraints (max velocity and acceleration)
   * @param updateFrequency The frequency at which the controller is updated
   */
  public TrapezoidalController(
      double kP,
      double kI,
      double kD,
      TrapezoidProfile.Constraints constraints,
      Frequency updateFrequency) {
    this.constraints = constraints;
    this.profile = new TrapezoidProfile(constraints);
    this.feedback = new PIDController(kP, kI, kD, updateFrequency.asPeriod().in(Seconds));
  }

  /**
   * Creates a new TrapezoidalController with the specified constraints and no feedback control. The difference between this
   * and a raw {@link TrapezoidProfile} is that this class supports stretching the profile to take a specified duration.
   * 
   * @param constraints The motion profile constraints (max velocity and acceleration)
   */
  public TrapezoidalController(
    TrapezoidProfile.Constraints constraints
  ) {
    this.constraints = constraints;
    this.profile = new TrapezoidProfile(constraints);
  }

  /**
   * Sets up a new motion profile from the given initial and goal states.
   *
   * @param initial The initial state (position and velocity when the motion starts)
   * @param goal The goal state (target position and velocity)
   */
  public void setProfile(State initial, State goal) {
    initialTime = Timer.getFPGATimestamp();

    this.initialState = initial;
    this.goalState = goal;

    initializeProfile(1.0);
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

  private void initializeProfile(double scalingFactor) {
    TrapezoidProfile.Constraints scaledConstraints = constraints;

    if (scalingFactor != 1.0) {
      scaledConstraints =
          new TrapezoidProfile.Constraints(
              constraints.maxVelocity * scalingFactor, constraints.maxAcceleration * scalingFactor);
    }

    profile = new TrapezoidProfile(scaledConstraints);
    profile.calculate(0, initialState, goalState);
  }

  /**
   * Gets the duration of the current motion profile, in seconds.
   *
   * @return The duration of the profile in seconds
   */
  public double getDuration() {
    return profile.totalTime();
  }

  /**
   * Stretches the current motion profile to take the specified duration.
   *
   * @param duration The desired duration of the profile in seconds
   */
  public void setDuration(double duration) {
    if (getDuration() == 0) {
      return;
    }

    // Upper bound is always 1
    double upperConstraintBound = 1.0;

    // Lower bound is found by repeatedly halving the constraints until the
    // profile duration is greater than the desired duration
    double lowerConstraintBound = 1.0;

    while (getDuration() <= duration) {
      lowerConstraintBound *= 0.5;
      initializeProfile(lowerConstraintBound);
    }

    // Now we have an upper and lower bound on the constraint scaling factor, so we can use
    // binary search to find the optimal scaling factor that gives us the desired duration
    while (upperConstraintBound - lowerConstraintBound > 0.0000001) {
      double midConstraintBound = (upperConstraintBound + lowerConstraintBound) / 2.;
      initializeProfile(midConstraintBound);
      if (Math.abs(sampleAt(0).position - initialState.position) > 0.001 ||
          Math.abs(sampleAt(0).velocity - initialState.velocity) > 0.001 ||
          Math.abs(sampleAt(getDuration()).position - goalState.position) > 0.001 ||
          Math.abs(sampleAt(getDuration()).velocity - goalState.velocity) > 0.001) {
        lowerConstraintBound = midConstraintBound;
      } else if (getDuration() < duration) {
        upperConstraintBound = midConstraintBound;
      } else {
        lowerConstraintBound = midConstraintBound;
      }
    }

    // Initialize the profile with the final scaling factor
    initializeProfile(upperConstraintBound);
  }

  /**
   * Samples the motion profile at the given time.
   *
   * @param time The time at which to sample the profile, in seconds
   * @return The state of the profile at the given time
   */
  public TrapezoidProfile.State sampleAt(double time) {
    return profile.calculate(time, initialState, goalState);
  }

  /**
   * Calculates the desired velocity at the current time, given the current state of the system.
   *
   * @param current The current state of the system (position and velocity)
   * @return The desired velocity that will cause the system to follow the motion profile
   */
  public double calculate(State current) {
    double time = Timer.getFPGATimestamp() - initialTime;

    if (time > getDuration()) {
      if (goalState.velocity != 0) {
        return goalState.velocity;
      } else if (feedback != null) {
        return feedback.calculate(current.position, goalState.position);
      } else {
        return 0.0;
      }
    } else {
      State profileState = sampleAt(time);
      double output = profileState.velocity; // Feedforward

      if (feedback != null) {
        output += feedback.calculate(current.position, profileState.position); // Feedback
      }

      return output;
    }
  }

  /**
   * Gets the target state of the profile at the current time.
   *
   * @return The target state at the current time
   */
  public TrapezoidProfile.State getCurrentTarget() {
    double time = Timer.getFPGATimestamp() - initialTime;
    return sampleAt(time);
  }

  /**
   * Gets the error between the current position of the system and the current target position of
   * the profile.
   *
   * @param currentPosition The current position of the system
   * @return The error between the current position and the target position
   */
  public double getError(double currentPosition) {
    State target = getCurrentTarget();
    return target.position - currentPosition;
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
   * @return The PID controller, which may be null if no feedback control is being used
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
    double time = Timer.getFPGATimestamp() - initialTime;
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
