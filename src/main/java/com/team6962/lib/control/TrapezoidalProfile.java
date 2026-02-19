package com.team6962.lib.control;

import com.team6962.lib.swerve.commands.DriveToStateCommand;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * A version of the trapezoidal motion profile that can be stretched to take a desired duration.
 * This is useful for cases where you want to follow a trapezoidal profile, but the duration of the
 * profile may need to be adjusted to synchronize multiple profiles. For example, this is a critical
 * component of {@link DriveToStateCommand}, which follows motion profiles on the direct, strafe,
 * and angular axes simultaneously, reaching the target state at the same time on all three axes.
 */
public class TrapezoidalProfile implements MotionProfile {
  /** The constraints for the trapezoidal motion profile. */
  private final TrapezoidProfile.Constraints constraints;

  /** The trapezoidal motion profile used to generate setpoints. */
  private TrapezoidProfile profile;

  /** The initial and goal states of the motion profile. */
  private MotionProfile.State initialState, goalState;

  /**
   * Creates a new StretchableTrapezoidalProfile with the specified constraints.
   *
   * @param constraints The motion profile constraints (max velocity and acceleration)
   */
  public TrapezoidalProfile(TrapezoidProfile.Constraints constraints) {
    this.constraints = constraints;
    this.profile = new TrapezoidProfile(constraints);
  }

  @Override
  public void setProfile(State initial, State goal) {
    this.initialState = initial;
    this.goalState = goal;

    initializeProfile(1.0);
  }

  private void initializeProfile(double scalingFactor) {
    TrapezoidProfile.Constraints scaledConstraints = constraints;

    if (scalingFactor != 1.0) {
      scaledConstraints =
          new TrapezoidProfile.Constraints(
              constraints.maxVelocity * scalingFactor, constraints.maxAcceleration * scalingFactor);
    }

    profile = new TrapezoidProfile(scaledConstraints);
    profile.calculate(0, initialState.toTrapezoidal(), goalState.toTrapezoidal());
  }

  @Override
  public double getDuration() {
    return profile.totalTime();
  }

  /**
   * Stretches the current motion profile to take the specified duration. Internally, this uses
   * optimization via binary search to find the optimal scaling factor for the constraints that
   * results in a profile with the desired duration, while still ensuring that the profile starts
   * and ends at the correct states.
   *
   * @param duration The desired duration of the profile in seconds
   */
  @Override
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
      if (Math.abs(sampleAt(0).position - initialState.position) > 0.001
          || Math.abs(sampleAt(0).velocity - initialState.velocity) > 0.001
          || Math.abs(sampleAt(getDuration()).position - goalState.position) > 0.001
          || Math.abs(sampleAt(getDuration()).velocity - goalState.velocity) > 0.001) {
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

  @Override
  public MotionProfile.State sampleAt(double time) {
    if (time < 0) {
      return initialState.extrapolate(time);
    } else if (time > getDuration()) {
      return goalState.extrapolate(time - getDuration());
    } else {
      return new MotionProfile.State(
          profile.calculate(time, initialState.toTrapezoidal(), goalState.toTrapezoidal()));
    }
  }

  /**
   * Gets the underlying trapezoidal motion profile.
   *
   * @return The trapezoidal motion profile
   */
  public TrapezoidProfile getProfile() {
    return profile;
  }
}
