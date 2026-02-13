package frc.robot;

import frc.robot.constants.EnabledSystems;

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
   * Whether to reorient the controls in simulation so that pressing the up key moves the robot up
   * the screen in the Sim GUI. This is recommended to be true when testing using the Sim GUI, but
   * should usually be false when visualizing with AdvantageScope's 3D field from a driver's
   * perspective.
   */
  public static final boolean reorientControlsInSimulation = true;

  /**
   * Which subsystems should be enabled. These subsystems will only be enabled if their
   * corresponding flag in enabledSystems is true and they exist on the robot according to the robot
   * constants. This is useful for testing individual subsystems without enabling the entire robot.
   *
   * <p>By default, all subsystems are enabled.
   */
  public static final EnabledSystems enabledSystems = new EnabledSystems();
}
