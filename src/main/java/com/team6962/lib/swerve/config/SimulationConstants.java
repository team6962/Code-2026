package com.team6962.lib.swerve.config;

import com.team6962.lib.swerve.CommandSwerveDrive;

/** The constants that define the behavior of the swerve drive in simulation. */
public class SimulationConstants {
  /**
   * If true, the pose estimation will be enabled in simulation, which fuses vision measurements
   * with odometry and gyroscope data to estimate the robot's pose. If false, {@link
   * CommandSwerveDrive#getPosition2d()} and similar methods will return the actual simulated robot
   * pose instead of an estimate of it.
   *
   * <p>This is recommended to be true when testing the vision system, but when experimenting with
   * autonomous routines it may be inconvenient.
   */
  public boolean EnablePoseEstimation = true;

  /**
   * Constructs a new SimulationConstants object with default values.
   */
  public SimulationConstants() {}

  /**
    * Sets whether pose estimation is enabled in simulation, and returns this SimulationConstants for
    * chaining.
    *
    * @param enablePoseEstimation Whether to enable pose estimation in simulation
    * @return This SimulationConstants object
    */
  public SimulationConstants withEnablePoseEstimation(boolean enablePoseEstimation) {
    EnablePoseEstimation = enablePoseEstimation;
    return this;
  }
}
