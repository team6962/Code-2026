package com.team6962.lib.swerve.simulation;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Gyroscope;
import com.team6962.lib.swerve.module.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Arrays;

/**
 * Simulates a complete swerve drivetrain including all modules and the gyroscope.
 *
 * <p>This is the top-level simulation class that orchestrates simulation of the entire swerve drive
 * system. It creates and manages {@link SwerveModuleSim} instances for each module and a {@link
 * GyroscopeSim}.
 *
 * <p>Getter methods provide access to the module and gyroscope simulations for inspection or direct
 * manipulation during testing.
 *
 * <p>The {@link #update(double)} method should be called periodically (typically every simulation
 * tick) to advance all subsystem simulations in the correct order.
 */
public class SwerveDriveSim {
  /** Simulation of the MapleSim arena */
  private MapleSim mapleSim;

  /** Simulations for each swerve module. */
  private SwerveModuleSim[] moduleSims;

  /** Simulation for the gyroscope. */
  private GyroscopeSim gyroscopeSim;

  /**
   * Creates a new swerve drive simulation.
   *
   * @param constants drivetrain configuration
   * @param modules the swerve modules to simulate
   * @param gyroscope the gyroscope to simulate
   */
  public SwerveDriveSim(
      DrivetrainConstants constants, SwerveModule[] modules, Gyroscope gyroscope) {
    mapleSim = new MapleSim(constants);

    this.moduleSims =
        Arrays.stream(modules)
            .map(module -> new SwerveModuleSim(module, mapleSim))
            .toArray(SwerveModuleSim[]::new);
    this.gyroscopeSim = new GyroscopeSim(constants, gyroscope, mapleSim);
  }

  /** Returns the array of module simulations. */
  public SwerveModuleSim[] getModules() {
    return moduleSims;
  }

  /** Returns the gyroscope simulation. */
  public GyroscopeSim getGyroscope() {
    return gyroscopeSim;
  }

  /** Returns the robot's current pose as computed by the MapleSim physics simulation. */
  public Pose2d getRobotPosition() {
    return mapleSim.getSwerveSim().getSimulatedDriveTrainPose();
  }

  /**
   * Returns the poses of all fuel game pieces currently in the arena simulation.
   *
   * @return an array of {@link Pose3d} representing the positions and orientations of all fuel game
   *     pieces currently in the arena simulation
   */
  public Pose3d[] getFuelPositions() {
    return mapleSim.getFuelPositions();
  }

  /**
   * Gets the MapleSim instance for direct access to arena and physics simulation data.
   *
   * @return the {@link MapleSim} instance representing the arena and physics simulation
   */
  public MapleSim getMapleSim() {
    return mapleSim;
  }

  /**
   * Updates all module and gyroscope simulations.
   *
   * @param deltaTimeSeconds time elapsed since the last update
   */
  public void update(double deltaTimeSeconds) {
    for (SwerveModuleSim moduleSim : moduleSims) {
      moduleSim.updateBeforeArena(deltaTimeSeconds);
    }

    mapleSim.update(deltaTimeSeconds);

    for (SwerveModuleSim moduleSim : moduleSims) {
      moduleSim.updateAfterArena(deltaTimeSeconds);
    }

    gyroscopeSim.update(deltaTimeSeconds);
  }
}
