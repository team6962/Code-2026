package com.team6962.lib.swerve.simulation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

/**
 * Simulates a swerve module's drive mechanism using MapleSim physics simulation.
 *
 * <p>This class bridges the gap between CTRE's TalonFX simulation state and MapleSim physics
 * simulation. The {@link #updateBeforeArena(double)} method passes the motor controller's applied
 * voltage to the physics simulation and the {@link #updateAfterArena(double)} method updates the
 * motor controller's simulated sensor values based on the physics simulation's results.
 *
 * <p>The simulation properly handles motor inversion by converting between the motor controller's
 * configured positive direction and the physics simulation's counter-clockwise-positive convention.
 *
 * <p>Getter methods return both angular values (wheel-relative) and linear values (movement of
 * wheel along ground) by converting with the configured wheel radius.
 *
 * <p>The {@link #updateBeforeArena(double)} and {@link #updateAfterArena(double)} methods should be
 * called periodically (typically every simulation tick) to advance the physics simulation and
 * synchronize state with the motor controller.
 */
public class DriveMechanismSim {
  /** Corner of the robot that this module occupies. */
  private Corner corner;

  /** Drivetrain configuration containing motor and wheel constants. */
  private DrivetrainConstants constants;

  /** CTRE simulation state for the TalonFX motor controller. */
  private TalonFXSimState motorControllerSimulation;

  /** MapleSim swerve module simulation instance for this drive mechanism. */
  private SwerveModuleSimulation moduleSim;

  /** MapleSim motor controller simulation instance for this drive mechanism. */
  private GenericMotorController motorSim;

  /**
   * Creates a new drive mechanism simulation.
   *
   * @param corner the corner of the robot that this module occupies
   * @param constants drivetrain configuration
   * @param motorController the TalonFX motor controller to simulate
   * @param mapleSim the MapleSim instance that simulates the swerve drive's physics
   */
  public DriveMechanismSim(
      Corner corner, DrivetrainConstants constants, TalonFX motorController, MapleSim mapleSim) {
    this.corner = corner;
    this.constants = constants;

    this.motorControllerSimulation = motorController.getSimState();
    this.moduleSim = mapleSim.getSwerveSim().getModules()[corner.getIndex()];

    motorSim = moduleSim.useGenericMotorControllerForDrive();
  }

  /**
   * Returns the sign multiplier to convert between the motor controller's configured positive
   * direction and the physics simulation's counter-clockwise-positive convention.
   *
   * @return -1.0 if the motor is configured as clockwise-positive, 1.0 if
   *     counter-clockwise-positive
   */
  private double getMotorInversionSign() {
    return constants.getDriveMotorConfig(corner).MotorOutput.Inverted
            == InvertedValue.Clockwise_Positive
        ? -1.0
        : 1.0;
  }

  /**
   * Passes the motor controller's applied voltage to the physics simulation before the arena
   * update.
   *
   * @param deltaTimeSeconds time elapsed since the last update
   */
  public void updateBeforeArena(double deltaTimeSeconds) {
    // Set the physics simulation input voltage to the motor controller's
    // applied output voltage
    motorSim.requestVoltage(
        motorControllerSimulation.getMotorVoltageMeasure().times(getMotorInversionSign()));
  }

  /**
   * Passes data from the physics simulation back to the motor controller simulation after the arena
   * update.
   *
   * @param deltaTimeSeconds time elapsed since the last update
   */
  public void updateAfterArena(double deltaTimeSeconds) {
    // Update the motor controller simulation with the new position,
    // velocity, and acceleration from the physics simulation
    motorControllerSimulation.setRawRotorPosition(
        moduleSim
            .getDriveWheelFinalPosition()
            .times(constants.DriveMotor.GearReduction * getMotorInversionSign()));
    motorControllerSimulation.setRotorVelocity(
        moduleSim
            .getDriveWheelFinalSpeed()
            .times(constants.DriveMotor.GearReduction * getMotorInversionSign()));
  }
}
