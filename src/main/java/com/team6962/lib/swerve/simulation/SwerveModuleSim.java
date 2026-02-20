package com.team6962.lib.swerve.simulation;

import com.team6962.lib.swerve.module.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Simulates a complete swerve module by combining drive and steer mechanism simulations.
 *
 * <p>This class composes {@link DriveMechanismSim} and {@link SteerMechanismSim} to provide a
 * unified simulation of a swerve module. It extracts the necessary hardware references from a real
 * {@link SwerveModule} to initialize the mechanism simulations.
 *
 * <p>Getter methods provide access to the individual mechanism simulations as well as combined
 * state in WPILib's {@link SwerveModulePosition} and {@link SwerveModuleState} formats for use with
 * kinematics and odometry.
 *
 * <p>The {@link #updateBeforeArena(double)} and {@link #updateAfterArena(double)} methods delegate
 * to both mechanism simulations and should be called periodically to advance the physics
 * simulation.
 */
public class SwerveModuleSim {
  /** Simulation for the drive mechanism. */
  private DriveMechanismSim driveMechanismSim;

  /** Simulation for the steer mechanism. */
  private SteerMechanismSim steerMechanismSim;

  /**
   * Creates a new swerve module simulation from a real module.
   *
   * @param module the swerve module to simulate
   */
  public SwerveModuleSim(SwerveModule module, MapleSim mapleSim) {
    driveMechanismSim =
        new DriveMechanismSim(
            module.getCorner(),
            module.getConstants(),
            module.getDriveMechanism().getMotorController(),
            mapleSim);
    steerMechanismSim =
        new SteerMechanismSim(
            module.getCorner(),
            module.getConstants(),
            module.getSteerMechanism().getMotorController(),
            module.getSteerMechanism().getEncoder(),
            mapleSim);
  }

  /** Returns the drive mechanism simulation. */
  public DriveMechanismSim getDriveMechanism() {
    return driveMechanismSim;
  }

  /** Returns the steer mechanism simulation. */
  public SteerMechanismSim getSteerMechanism() {
    return steerMechanismSim;
  }

  /**
   * Updates both mechanism simulations before the arena update, passing the motor controller
   * outputs to the physics simulation.
   *
   * @param deltaTimeSeconds time elapsed since the last update
   */
  public void updateBeforeArena(double deltaTimeSeconds) {
    driveMechanismSim.updateBeforeArena(deltaTimeSeconds);
    steerMechanismSim.updateBeforeArena(deltaTimeSeconds);
  }

  /**
   * Updates both mechanism simulations after the arena update, passing data from the physics
   * simulation back to the motor controller simulations.
   *
   * @param deltaTimeSeconds time elapsed since the last update
   */
  public void updateAfterArena(double deltaTimeSeconds) {
    driveMechanismSim.updateAfterArena(deltaTimeSeconds);
    steerMechanismSim.updateAfterArena(deltaTimeSeconds);
  }
}
