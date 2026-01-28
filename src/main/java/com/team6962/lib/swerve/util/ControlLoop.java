package com.team6962.lib.swerve.util;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Consumer;

/**
 * A control loop abstraction that provides periodic execution of an update function at a
 * configurable frequency.
 *
 * <p>This interface allows different control loop implementations to be swapped without changing
 * the calling code. The update function receives the actual delta time since the last update,
 * enabling time-based calculations that remain accurate even with timing jitter.
 *
 * <p>Two implementations are provided:
 *
 * <ul>
 *   <li>{@link SubsystemPeriodic} - Runs on the main robot thread at 50 Hz
 *   <li>{@link Threaded} - Runs on a dedicated thread at any frequency
 * </ul>
 */
public interface ControlLoop extends AutoCloseable {
  /**
   * Starts the control loop with the given update function and frequency.
   *
   * @param updateFunction function to call each period, receives delta time
   * @param updateFrequency how often to call the update function
   */
  public default void start(Consumer<Double> updateFunction, Frequency updateFrequency) {}

  @Override
  public default void close() {}

  /**
   * A control loop implementation that runs on the main robot thread by extending {@link
   * SubsystemBase} and using its periodic method.
   *
   * <p>This implementation only supports 50 Hz (the robot's main loop frequency). Other frequencies
   * are ignored with a warning. Use {@link Threaded} if a different frequency is required.
   */
  public static class SubsystemPeriodic extends SubsystemBase implements ControlLoop {
    private Consumer<Double> updateFunction;
    private double lastUpdateTimestamp = -1;

    @Override
    public void start(Consumer<Double> updateFunction, Frequency updateFrequency) {
      this.updateFunction = updateFunction;

      if (!updateFrequency.isEquivalent(Hertz.of(50))) {
        System.err.println(
            "Warning: RobotPeriodicLoop only supports 50 Hz update frequency. Ignoring provided frequency.");
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

  /**
   * A control loop implementation that runs on a dedicated thread using WPILib's {@link Notifier}.
   *
   * <p>This implementation supports any update frequency and automatically handles timing drift by
   * tracking a goal timestamp. If execution falls behind schedule, it will skip periods to catch up
   * rather than queuing multiple rapid updates.
   *
   * <p>The thread runs at real-time priority for consistent timing. Call {@link #close()} when done
   * to release the thread resources.
   */
  public static class Threaded implements ControlLoop {
    private Consumer<Double> updateFunction;
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
      lastUpdateTimestamp = -1;
      goalUpdateTimestamp = -1;
      notifier.close();
    }
  }
}
