package com.team6962.lib.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;
import com.team6962.lib.swerve.util.SwerveComponent;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Current;

/**
 * Represents a single swerve module, which controls a wheel using of a drive motor, a steer motor,
 * and an absolute encoder on the steer axis.
 *
 * <p>This class coordinates the {@link DriveMechanism} and {@link SteerMechanism} components,
 * providing a unified interface for:
 *
 * <ul>
 *   <li>Reading module state (velocity and angle)
 *   <li>Reading module position (distance traveled and angle)
 *   <li>Applying control requests to both motors
 *   <li>Logging telemetry data
 *   <li>Updating status signals
 * </ul>
 *
 * <p>Each swerve module is associated with a {@link Corner} (front-left, front-right, back-left, or
 * back-right) which determines its position on the robot, its configuration from the drivetrain
 * constants, and the index in arrays that data associated with the module is stored in.
 *
 * @see DriveMechanism
 * @see SteerMechanism
 * @see Corner
 */
public class SwerveModule implements SwerveComponent, AutoCloseable {
  /** The corner of the robot that this module occupies. */
  private Corner corner;

  /** The drivetrain constants containing configuration for this module. */
  private DrivetrainConstants constants;

  /** The drive mechanism controlling wheel rotation around its shaft. */
  private DriveMechanism driveMechanism;

  /** The steer mechanism controlling wheel rotation around a vertical axis. */
  private SteerMechanism steerMechanism;

  /**
   * Creates a new SwerveModule at the specified corner with the given configuration.
   *
   * @param corner The corner of the robot that this module occupies (front-left, front-right, etc.)
   * @param constants The drivetrain constants used to configure the module
   */
  public SwerveModule(Corner corner, DrivetrainConstants constants) {
    this.corner = corner;
    this.constants = constants;

    driveMechanism = new DriveMechanism(corner, constants);
    steerMechanism = new SteerMechanism(corner, constants);
  }

  /**
   * Gets the status signals used by this swerve module. These signals will be periodically
   * refreshed by the containing class.
   *
   * @return An array of the status signals used by this module
   */
  @Override
  public BaseStatusSignal[] getStatusSignals() {
    return SwerveComponent.combineStatusSignals(driveMechanism, steerMechanism);
  }

  /**
   * Gets the list of Phoenix devices connected to by this component. All devices in this list will
   * have their bus utilization optimized in parallel.
   *
   * @return an array of Phoenix devices used by this component
   */
  @Override
  public ParentDevice[] getPhoenixDevices() {
    return SwerveComponent.combinePhoenixDevices(driveMechanism, steerMechanism);
  }

  /**
   * Updates the module state.
   *
   * @param deltaTimeSeconds The time elapsed since the last update
   */
  @Override
  public void update(double deltaTimeSeconds) {
    driveMechanism.update(deltaTimeSeconds);
    steerMechanism.update(deltaTimeSeconds);
  }

  /**
   * Gets the corner of the robot that this module occupies.
   *
   * @return The corner (front-left, front-right, back-left, or back-right)
   */
  public Corner getCorner() {
    return corner;
  }

  /**
   * Gets the index of this module (0-3 corresponding to the corner).
   *
   * @return The module index
   */
  public int getIndex() {
    return corner.getIndex();
  }

  /**
   * Gets the drivetrain constants used to configure this module.
   *
   * @return The drivetrain constants
   */
  public DrivetrainConstants getConstants() {
    return constants;
  }

  /**
   * Gets the drive mechanism for direct access to drive motor functions.
   *
   * @return The drive mechanism
   */
  public DriveMechanism getDriveMechanism() {
    return driveMechanism;
  }

  /**
   * Gets the steer mechanism for direct access to steer motor functions.
   *
   * @return The steer mechanism
   */
  public SteerMechanism getSteerMechanism() {
    return steerMechanism;
  }

  /**
   * Logs telemetry data for this swerve module.
   *
   * @param basePath The base path that all data will be logged under
   */
  @Override
  public void logTelemetry(String basePath) {
    if (!basePath.endsWith("/")) {
      basePath += "/";
    }

    driveMechanism.logTelemetry(basePath + "Drive/");
    steerMechanism.logTelemetry(basePath + "Steer/");
  }

  /**
   * Gets the current state of the module (drive velocity and steer angle).
   *
   * @return The current module state as a {@link SwerveModuleState}
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMechanism.getVelocity(), new Rotation2d(steerMechanism.getPosition()));
  }

  /**
   * Gets the current position of the module (distance traveled and steer angle).
   *
   * @return The current module position as a {@link SwerveModulePosition}
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMechanism.getPosition(), new Rotation2d(steerMechanism.getPosition()));
  }

  /**
   * Gets the total supply current draw of the swerve module.
   *
   * @return The total supply current draw.
   */
  public Current getSupplyCurrent() {
    return driveMechanism.getSupplyCurrent().plus(steerMechanism.getSupplyCurrent());
  }

  /**
   * Sets the control requests for the drive and steer motors. This is the intended method for
   * motions to command the module to achieve a desired state.
   *
   * @param driveRequest The control request for the drive motor
   * @param steerRequest The control request for the steer motor
   */
  public void setControl(ControlRequest driveRequest, ControlRequest steerRequest) {
    driveMechanism.setControl(driveRequest);
    steerMechanism.setControl(steerRequest);
  }

  /** Closes the module and releases underlying resources. */
  @Override
  public void close() {
    driveMechanism.close();
    steerMechanism.close();
  }
}
