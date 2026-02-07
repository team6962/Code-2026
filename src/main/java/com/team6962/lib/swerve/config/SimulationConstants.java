package com.team6962.lib.swerve.config;

import com.team6962.lib.swerve.CommandSwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
   * The number of physics sub-ticks to simulate per control loop period. Higher values improve
   * simulation accuracy at the cost of computational performance. A value of 1 means the physics
   * simulation runs once per swerve control loop period. Values of 2-5 provide more accurate
   * physics for fast movements or collisions.
   */
  public int SimulationSubTicksPerPeriod = 1;

  /**
   * If true, enables collision detection between the robot or fuel and field ramps. When enabled,
   * the robot and fuel are physically blocked by the ramps. When false, the objects pass through
   * the ramps without collision. This is recommended to be false in most cases.
   */
  public boolean EnableRampCollider = false;

  /**
   * If true, enables efficiency mode in the MapleSim arena to reduce CPU usage. This decreases the
   * number of fuel game pieces on the field.
   */
  public boolean EnableEfficiencyMode = false;

  /** The initial pose (position and rotation) of the robot when the simulation starts. */
  public Pose2d InitialPose = new Pose2d(2, 2, new Rotation2d());

  /** Constructs a new SimulationConstants object with default values. */
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

  /**
   * Sets the number of physics sub-ticks per control loop period, and returns this
   * SimulationConstants for chaining. Higher values improve accuracy at the cost of performance.
   *
   * @param simulationSubTicksPerPeriod The number of sub-ticks to simulate per swerve control loop
   *     period (1-5 recommended)
   * @return This SimulationConstants object
   */
  public SimulationConstants withSimulationSubTicksPerPeriod(int simulationSubTicksPerPeriod) {
    SimulationSubTicksPerPeriod = simulationSubTicksPerPeriod;
    return this;
  }

  /**
   * Sets whether ramp collision detection is enabled for the robot and fuel, and returns this
   * SimulationConstants for chaining. Recommended to be false in most cases.
   *
   * @param enableRampCollider Whether to enable collision detection with field ramps
   * @return This SimulationConstants object
   */
  public SimulationConstants withEnableRampCollider(boolean enableRampCollider) {
    EnableRampCollider = enableRampCollider;
    return this;
  }

  /**
   * Sets whether efficiency mode is enabled to reduce CPU usage by decreasing the number of fuel
   * game pieces on the field, and returns this SimulationConstants for chaining.
   *
   * @param enableEfficiencyMode Whether to enable efficiency mode
   * @return This SimulationConstants object
   */
  public SimulationConstants withEnableEfficiencyMode(boolean enableEfficiencyMode) {
    EnableEfficiencyMode = enableEfficiencyMode;
    return this;
  }

  /**
   * Sets the initial pose of the robot when simulation starts, and returns this SimulationConstants
   * for chaining.
   *
   * @param initialPose The starting position and rotation of the robot
   * @return This SimulationConstants object
   */
  public SimulationConstants withInitialPose(Pose2d initialPose) {
    InitialPose = initialPose;
    return this;
  }
}
