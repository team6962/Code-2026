package frc.robot;

import frc.robot.constants.CompetitionBotConstants;
import frc.robot.constants.RobotConstants;

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
   * The robot constants to use in simulation. This should be set to a constants class that
   * corresponds to the robot being simulated. This is used to determine which subsystems exist on
   * the simulated robot and to provide the correct configuration values for those subsystems.
   */
  public static final RobotConstants simulatedRobot = new CompetitionBotConstants();
}
