package com.team6962.lib.swerve.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.MomentOfInertia;

/**
 * The constants for a unique type of swerve module, different from the default ones whose
 * properties are specified in {@link DrivetrainConstants}. This class allows the use of multiple
 * different types of swerve modules in the same drivetrain, each with their own unique properties:
 *
 * <ul>
 *   <li>Steer gear reduction
 *   <li>Steer motor configuration
 *   <li>Simulated moment of inertia of the steer mechanism
 *   <li>Drive motor configuration
 * </ul>
 *
 * <p>If any of these properties are not specified for a module, the corresponding property from the
 * swerve drive's configuration will be used instead. This allows you to specify only the properties
 * that are different for a module, and use the default values from the swerve drive's configuration
 * for the rest.
 */
public class UniqueModuleConstants {
  /**
   * The gear reduction from the steer motor to the steer mechanism. When 0, the gear reduction from
   * the swerve drive's configuration will be used instead.
   */
  public double SteerGearReduction = 0;

  /**
   * The TalonFX configuration for the steer motor of this module. When null, the TalonFX
   * configuration from the swerve drive's configuration will be used instead.
   */
  public TalonFXConfiguration SteerMotorConfig;

  /**
   * The moment of inertia of the steer mechanism for simulation. This includes the rotor inertia
   * plus any additional inertia from the gearing. When null, the moment of inertia from the swerve
   * drive's configuration will be used instead.
   */
  public MomentOfInertia SteerMomentOfInertia = null;

  /**
   * The TalonFX configuration for the drive motor of this module. When null, the TalonFX
   * configuration from the swerve drive's configuration will be used instead.
   */
  public TalonFXConfiguration DriveMotorConfig;

  /** Constructs a new UniqueModuleConstants object with default values. */
  public UniqueModuleConstants() {}

  /**
   * Sets the steer gear reduction for this module, and returns this UniqueModuleConstants for
   * chaining.
   *
   * @param steerGearReduction The gear reduction from the steer motor to the steer mechanism
   * @return This UniqueModuleConstants object
   */
  public UniqueModuleConstants withSteerGearReduction(double steerGearReduction) {
    SteerGearReduction = steerGearReduction;
    return this;
  }

  /**
   * Sets the TalonFX configuration for the steer motor of this module, and returns this
   * UniqueModuleConstants for chaining.
   *
   * @param steerMotorConfig The TalonFX configuration for the steer motor
   * @return This UniqueModuleConstants object
   */
  public UniqueModuleConstants withSteerMotorConfig(TalonFXConfiguration steerMotorConfig) {
    SteerMotorConfig = steerMotorConfig;
    return this;
  }

  /**
   * Sets the moment of inertia of the steer mechanism for simulation, and returns this
   * UniqueModuleConstants for chaining.
   *
   * @param steerMomentOfInertia The moment of inertia including rotor and gearing
   * @return This UniqueModuleConstants object
   */
  public UniqueModuleConstants withSteerMomentOfInertia(MomentOfInertia steerMomentOfInertia) {
    SteerMomentOfInertia = steerMomentOfInertia;
    return this;
  }

  /**
   * Sets the TalonFX configuration for the drive motor of this module, and returns this
   * UniqueModuleConstants for chaining.
   *
   * @param driveMotorConfig The TalonFX configuration for the drive motor
   * @return This UniqueModuleConstants object
   */
  public UniqueModuleConstants withDriveMotorConfig(TalonFXConfiguration driveMotorConfig) {
    DriveMotorConfig = driveMotorConfig;
    return this;
  }
}
