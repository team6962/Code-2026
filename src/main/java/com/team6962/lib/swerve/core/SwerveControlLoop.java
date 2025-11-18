package com.team6962.lib.swerve.core;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveControlLoop extends Subsystem, AutoCloseable {
    public default void lockResources() {
    }

    public default void unlockResources() {
    }

    public default void start(Consumer<Double> updateFunction, Frequency updateFrequency) {
    }

    @Override
    public default void close() {
    }

    public static class RobotPeriodic implements SwerveControlLoop {
        private Consumer<Double> updateFunction;
        private double lastUpdateTimestamp = -1;

        @Override
        public void start(Consumer<Double> updateFunction, Frequency updateFrequency) {
            this.updateFunction = updateFunction;

            if (!updateFrequency.isEquivalent(Hertz.of(50))) {
                System.err.println("Warning: RobotPeriodicLoop only supports 50 Hz update frequency. Ignoring provided frequency.");
            }
        }

        @Override
        public void periodic() {
            if (lastUpdateTimestamp < 0) {
                lastUpdateTimestamp = Timer.getFPGATimestamp();
                return;
            }

            double currentTimestamp = Timer.getFPGATimestamp();
            double deltaTime = currentTimestamp - lastUpdateTimestamp;
            lastUpdateTimestamp = currentTimestamp;

            updateFunction.accept(deltaTime);
        }
    }

    public static class Threaded implements SwerveControlLoop {
        private Consumer<Double> updateFunction;
        private ReentrantLock resourceLock = new ReentrantLock();
        private Notifier notifier;
        private double updatePeriodSeconds;
        private double lastUpdateTimestamp = -1;
        private double goalUpdateTimestamp = -1;

        @Override
        public void start(Consumer<Double> updateFunction, Frequency updateFrequency) {
            this.updateFunction = updateFunction;
            notifier = new Notifier(this::threadedPeriodic);
            updatePeriodSeconds = updateFrequency.asPeriod().in(Seconds);
            notifier.startSingle(0);
        }

        private void threadedPeriodic() {
            if (lastUpdateTimestamp < 0) {
                lastUpdateTimestamp = Timer.getFPGATimestamp();
                goalUpdateTimestamp = lastUpdateTimestamp + updatePeriodSeconds;
                notifier.startSingle(updatePeriodSeconds);
                return;
            }

            // Compute the actual delta time since the last update
            double currentTimestamp = Timer.getFPGATimestamp();
            double deltaTime = currentTimestamp - lastUpdateTimestamp;
            lastUpdateTimestamp = currentTimestamp;

            // Run the update function
            updateFunction.accept(deltaTime);

            // Schedule the next update
            currentTimestamp = Timer.getFPGATimestamp();

            goalUpdateTimestamp += updatePeriodSeconds;

            // If we're far behind schedule, delay one or more periods
            while (goalUpdateTimestamp < currentTimestamp) {
                goalUpdateTimestamp += updatePeriodSeconds;
            }

            double timeUntilNextUpdate = goalUpdateTimestamp - currentTimestamp;

            notifier.startSingle(timeUntilNextUpdate);
        }

        @Override
        public void close() {
            notifier.close();
            resourceLock.unlock();
            lastUpdateTimestamp = -1;
            goalUpdateTimestamp = -1;
        }

        @Override
        public void lockResources() {
            resourceLock.lock();
        }

        @Override
        public void unlockResources() {
            resourceLock.unlock();
        }
    }
}
