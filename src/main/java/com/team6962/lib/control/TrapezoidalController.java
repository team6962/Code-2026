package com.team6962.lib.control;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Timer;

public class TrapezoidalController {
    private TrapezoidProfile profile;
    private PIDController feedback;
    private double initialTime;
    private TrapezoidProfile.State initialState, goalState;

    /**
     * A scaling factor for the time duration of the profile. This value should
     * always be greater than or equal to 1, making the profile take longer than
     * or equal to the minimum time required by the physical constraints of the
     * system.
     */
    private double timeScale = 1.0;

    public TrapezoidalController(
        double kP, double kI, double kD,
        TrapezoidProfile.Constraints constraints,
        Frequency updateFrequency
    ) {
        this.profile = new TrapezoidProfile(constraints);
        this.feedback = new PIDController(kP, kI, kD, updateFrequency.asPeriod().in(Seconds));
    }

    public void setProfile(State initial, State goal) {
        timeScale = 1.0;
        initialTime = Timer.getFPGATimestamp();

        this.initialState = initial;
        this.goalState = goal;

        profile.calculate(0, initial, goal);
    }

    public void setProfile(State initial, State goal, double duration) {
        setProfile(initial, goal);
        setDuration(duration);
    }

    public double getDuration() {
        return profile.totalTime() * timeScale;
    }

    public void setDuration(double duration) {
        double unscaledDuration = profile.totalTime();
        timeScale = duration / unscaledDuration;
    }

    private TrapezoidProfile.State sampleAt(double time) {
        TrapezoidProfile.State originalState = profile.calculate(time / timeScale, initialState, goalState);

        return new TrapezoidProfile.State(
            originalState.position,
            originalState.velocity / timeScale
        );
    }

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

    public TrapezoidProfile getProfile() {
        return profile;
    }

    public PIDController getFeedback() {
        return feedback;
    }

    public double getRemainingTime() {
        double time = (Timer.getFPGATimestamp() - initialTime) / timeScale;
        return Math.max(0, getDuration() - time);
    }

    public boolean isFinished() {
        return getRemainingTime() <= 0;
    }
}
