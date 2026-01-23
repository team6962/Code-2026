package com.team6962.lib.control;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Timer;

/**
 * A controller that can be used to follow trapezoidal motion profiles with PID
 * feedback. This controller is designed to work at a high level, generating
 * velocity setpoints to be sent to a lower-level velocity controller.
 * 
 * <h3>Usage</h3>
 * 
 * To use this controller, first create an instance of the
 * {@link SimpleTrapezoidalController} class, providing the PID constants, motion
 * profile constraints (maximum velocity and acceleration), and update
 * frequency. The {@link #calculate calculate()} method can then be called periodically to
 * get the desired velocity that will move the system along the profile.
 * <p>
 * The {@link #getDuration()} method can be used to get the expected
 * total time the profile will take to complete (the time it will take the
 * system to reach the goal state from the initial state). The
 * {@link #getRemainingTime()} method can be used to get the
 * estimated time until the profile will be completed. The {@link #isFinished()}
 * method can be used to check if the profile has been completed.
 * <p>
 * Note that this controller does not support time scaling; to synchronize
 * multiple controllers, the user must manually adjust the motion profile
 * constraints to ensure they all have the same duration or using
 * {@link SynchronizableTrapezoidalController} instead.
 */
public class SimpleTrapezoidalController {
    /**
     * The trapezoidal motion profile used to generate setpoints.
     */
    private final TrapezoidProfile profile;

    /**
     * The PID controller used for feedback control.
     */
    private final PIDController feedback;

    /**
     * The initial time when the profile started, in seconds since the FPGA
     * started.
     */
    private double initialTime;
    
    /**
     * The initial and state of the motion profile.
     */
    private TrapezoidProfile.State profileInitialState;

    /**
     * The goal state of the motion profile.
     */
    private TrapezoidProfile.State profileGoalState;

    public SimpleTrapezoidalController(
        double kP, double kI, double kD,
        TrapezoidProfile.Constraints constraints,
        Frequency updateFrequency
    ) {
        this.profile = new TrapezoidProfile(constraints);
        this.feedback = new PIDController(kP, kI, kD, updateFrequency.asPeriod().in(Seconds));
    }

    /**
     * Sets up a new motion profile from the given initial and goal states.
     * 
     * @param initial The initial state (position and velocity when the motion
     * starts)
     * @param goal The goal state (target position and velocity)
     */
    private void setProfile(State initial, State goal) {
        initialTime = Timer.getFPGATimestamp();

        this.profileInitialState = initial;
        this.profileGoalState = goal;

        profile.calculate(0, initial, goal);
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
     * Samples the motion profile at the given time to find the expected state.
     * 
     * @param time The time at which to sample the profile, in seconds
     * @return The state of the profile at the given time
     */
    private TrapezoidProfile.State sampleAt(double time) {
        TrapezoidProfile.State originalState = profile.calculate(time, profileInitialState, profileGoalState);

        return new TrapezoidProfile.State(
            originalState.position,
            originalState.velocity
        );
    }

    /**
     * Calculates the desired velocity at the current time, given the current
     * state of the system and desired goal state.
     * 
     * @param current The current state of the system (position and velocity)
     * @param goal The desired goal state (position and velocity)
     * @return The desired velocity that will cause the system to follow the
     * motion profile
     */
    public double calculate(State current, State goal) {
        if (profileGoalState == null || goal.position != profileGoalState.position || goal.velocity != profileGoalState.velocity || isFinished()) {
            setProfile(current, goal);
        }

        double time = (Timer.getFPGATimestamp() - initialTime);

        if (time > getDuration()) {
            if (profileGoalState.velocity != 0) {
                return profileGoalState.velocity;
            } else {
                return feedback.calculate(current.position, profileGoalState.position);
            }
        } else {
            State profileState = sampleAt(time);

            double feedbackOutput = feedback.calculate(current.position, profileState.position);
            double feedforward = profileState.velocity;

            return feedbackOutput + feedforward;
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
