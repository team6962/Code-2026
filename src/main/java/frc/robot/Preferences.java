package frc.robot;

import com.team6962.lib.swerve.config.DrivetrainConstants;

public class Preferences {
  /**
   * Whether to use pose estimation when running in simulation. When true, the estimated robot pose
   * using vision, odometry, and gyroscope data will be shown instead of the actual robot pose. When
   * false, the robot's simulated pose will be accessible directly.
   *
   * <p>This is recommended to be true when testing the vision system, but when experimenting with
   * autonomous routines it may be inconvenient.
   */
  public static final boolean enablePoseEstimationInSimulation = false;

  /**
   * Applies the preferences to the given drivetrain constants.
   *
   * @param constants The drivetrain constants to apply the preferences to.
   * @return The updated drivetrain constants.
   */
  public static DrivetrainConstants apply(DrivetrainConstants constants) {
    constants.Simulation.EnablePoseEstimation = enablePoseEstimationInSimulation;
    return constants;
  }
}
