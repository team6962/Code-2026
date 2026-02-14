package com.team6962.lib.control;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Timer;

/**
 * A controller that can be used to follow strafe velocity motion profiles with PID feedback. This
 * controller is designed to work at a high level, generating velocity setpoints to be sent to a
 * lower-level velocity controller.
 *
 * <h3>Usage</h3>
 *
 * To use this controller, first create an instance of the {@link StrafeVelocityController} class,
 * providing the PID constants, motion profile constraints (maximum velocity and acceleration), and
 * update frequency. Next, set up a motion profile by calling the {@link #setProfile setProfile()}
 * method with the initial velocity, target velocity, and total duration. The {@link #calculate
 * calculate()} method can then be called periodically to get the desired velocity that will move
 * the system along the profile.
 *
 * <p>The {@link #getDuration()} method can be used to get the expected total time the profile will
 * take to complete. The {@link #getRemainingTime()} method can be used to get the estimated time
 * until the profile will be completed. The {@link #isFinished()} method can be used to check if the
 * profile has been completed.
 */
public class StrafeVelocityController {
  /** The strafe velocity motion profile used to generate setpoints. */
  private final StrafeVelocityProfile profile;

  /** The PID controller used for feedback control. */
  private final PIDController feedback;

  /** The initial time when the profile started, in seconds since the FPGA started. */
  private double initialTime;

  /** The initial and target velocities of the motion profile. */
  private double initialVelocity, targetVelocity;

  /** The total duration of the profile. */
  private double duration;

  /**
   * Creates a new StrafeVelocityController with the specified PID gains, constraints, and update
   * frequency.
   *
   * @param kP The proportional gain
   * @param kI The integral gain
   * @param kD The derivative gain
   * @param constraints The motion profile constraints (max velocity and acceleration)
   * @param updateFrequency The frequency at which the controller is updated
   */
  public StrafeVelocityController(
      double kP,
      double kI,
      double kD,
      TrapezoidProfile.Constraints constraints,
      Frequency updateFrequency) {
    this.profile = new StrafeVelocityProfile(constraints);
    this.feedback = new PIDController(kP, kI, kD, updateFrequency.asPeriod().in(Seconds));
  }

  /**
   * Sets up a new motion profile from the given initial velocity, target velocity, and duration.
   *
   * @param initialVelocity The initial strafe velocity when the motion starts
   * @param targetVelocity The target strafe velocity at the end
   * @param duration The total duration of the profile in seconds
   */
  public void setProfile(double initialVelocity, double targetVelocity, double duration) {
    this.initialTime = Timer.getFPGATimestamp();
    this.initialVelocity = initialVelocity;
    this.targetVelocity = targetVelocity;
    this.duration = duration;
  }

  /**
   * Sets the duration of the current motion profile. This can be used to stretch the profile to
   * make it take longer than the minimum time required by the physical constraints of the system.
   *
   * @param duration The desired duration of the profile in seconds
   */
  public void setDuration(double duration) {
    this.duration = duration;
  }

  /**
   * Gets the duration of the current motion profile, in seconds.
   *
   * @return The duration of the profile in seconds
   */
  public double getDuration() {
    return duration;
  }

  /**
   * Samples the motion profile at the given time.
   *
   * @param time The time at which to sample the profile, in seconds
   * @return The state of the profile at the given time
   */
  private TrapezoidProfile.State sampleAt(double time) {
    return profile.calculate(initialVelocity, targetVelocity, time, duration);
  }

  /**
   * Calculates the desired velocity at the current time, given the current state of the system.
   *
   * @param currentState The current strafe position of the system
   * @return The desired velocity that will cause the system to follow the motion profile
   */
  public double calculate(TrapezoidProfile.State currentState) {
    double realTime = Timer.getFPGATimestamp() - initialTime;

    if (realTime > getDuration()) {
      if (targetVelocity != 0) {
        return targetVelocity;
      } else {
        TrapezoidProfile.State finalState = sampleAt(duration);
        return feedback.calculate(currentState.position, finalState.position);
      }
    } else {
      TrapezoidProfile.State profileState = sampleAt(realTime);

      double feedbackOutput = feedback.calculate(currentState.position, profileState.position);
      double feedforward = profileState.velocity;

      return feedbackOutput + feedforward;
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
    TrapezoidProfile.State target = getCurrentTarget();
    return target.position - currentPosition;
  }

  /**
   * Gets the underlying strafe velocity motion profile.
   *
   * @return The strafe velocity motion profile
   */
  public StrafeVelocityProfile getProfile() {
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
