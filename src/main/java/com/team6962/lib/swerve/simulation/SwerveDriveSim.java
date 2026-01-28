package com.team6962.lib.swerve.simulation;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Gyroscope;
import com.team6962.lib.swerve.module.SwerveModule;
import java.util.Arrays;

/**
 * Simulates a complete swerve drivetrain including all modules and the gyroscope.
 *
 * <p>This is the top-level simulation class that orchestrates simulation of the entire swerve drive
 * system. It creates and manages {@link SwerveModuleSim} instances for each module and a {@link
 * GyroscopeSim} that derives heading from module motion.
 *
 * <p>Getter methods provide access to the module and gyroscope simulations for inspection or direct
 * manipulation during testing.
 *
 * <p>The {@link #update(double)} method should be called periodically (typically every simulation
 * tick) to advance all subsystem simulations in the correct order: modules first, then gyroscope
 * (which depends on module positions).
 */
public class SwerveDriveSim {
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
    this.moduleSims =
        Arrays.stream(modules)
            .map(module -> new SwerveModuleSim(module))
            .toArray(SwerveModuleSim[]::new);
    this.gyroscopeSim = new GyroscopeSim(constants, gyroscope, moduleSims);
  }

  /** Returns the array of module simulations. */
  public SwerveModuleSim[] getModules() {
    return moduleSims;
  }

  /** Returns the gyroscope simulation. */
  public GyroscopeSim getGyroscope() {
    return gyroscopeSim;
  }

  /**
   * Updates all module and gyroscope simulations.
   *
   * @param deltaTimeSeconds time elapsed since the last update
   */
  public void update(double deltaTimeSeconds) {
    // Update each module simulation
    for (SwerveModuleSim moduleSim : moduleSims) {
      moduleSim.update(deltaTimeSeconds);
    }

    // Update gyroscope simulation
    gyroscopeSim.update(deltaTimeSeconds);
  }
}
