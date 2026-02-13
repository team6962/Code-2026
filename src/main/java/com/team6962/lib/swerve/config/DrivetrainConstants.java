package com.team6962.lib.swerve.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

/** A set of constants that configure the behavior of a swerve drivetrain. */
public class DrivetrainConstants implements Cloneable {
  /**
   * The name of the CAN bus that the drivetrain's devices are connected to. This defaults to an
   * empty string, which means the RIO CAN bus on a RoboRIO. If your drivetrain is using a different
   * CAN bus, set this to the name of the canivore.
   */
  public String CANBusName = "";

  /**
   * The constants that define the behavior of the gyroscope used for determining the heading of the
   * robot.
   */
  public GyroscopeConstants Gyroscope = new GyroscopeConstants();

  /**
   * The constants that define various update frequencies and whether to use timesync for control
   * requests.
   */
  public TimingConstants Timing = new TimingConstants();

  /** The constants that define the drive motor behavior. */
  public DriveMotorConstants DriveMotor = new DriveMotorConstants();

  /** The constants that define the steer motor behavior. */
  public SteerMotorConstants SteerMotor = new SteerMotorConstants();

  /** The constants that define the steer encoder behavior. */
  public SteerEncoderConstants SteerEncoder = new SteerEncoderConstants();

  /**
   * The constants for each of the swerve modules in the drivetrain, which include the CAN IDs of
   * each device, absolute encoder offsets, and optional wheel radii.
   */
  public SwerveModuleConstants[] SwerveModules = new SwerveModuleConstants[4];

  /** The constants that define the driving behavior of the drivetrain. */
  public DrivingConstants Driving = new DrivingConstants();

  /** The constants that define the physical structure of the drivetrain. */
  public StructureConstants Structure = new StructureConstants();

  /** The constants that define the behavior of the swerve drive in simulation. */
  public SimulationConstants Simulation = new SimulationConstants();

  public DrivetrainConstants() {}

  /**
   * Sets the name of the CAN bus that the drivetrain's devices are connected to, and returns this
   * DrivetrainConstants for chaining.
   *
   * @param canBusName The name of the CAN bus
   * @return This DrivetrainConstants object
   */
  public DrivetrainConstants withCANBusName(String canBusName) {
    CANBusName = canBusName;
    return this;
  }

  /**
   * Sets the structure constants for the drivetrain, and returns this DrivetrainConstants for
   * chaining.
   *
   * @param structure The structure constants
   * @return This DrivetrainConstants object
   */
  public DrivetrainConstants withStructure(StructureConstants structure) {
    Structure = structure;
    return this;
  }

  /**
   * Sets the gyroscope constants for the drivetrain, and returns this DrivetrainConstants for
   * chaining.
   *
   * @param gyroscope The gyroscope constants
   * @return This DrivetrainConstants object
   */
  public DrivetrainConstants withGyroscope(GyroscopeConstants gyroscope) {
    Gyroscope = gyroscope;
    return this;
  }

  /**
   * Sets the timing constants for the drivetrain, and returns this DrivetrainConstants for
   * chaining.
   *
   * @param timing The timing constants
   * @return This DrivetrainConstants object
   */
  public DrivetrainConstants withTiming(TimingConstants timing) {
    Timing = timing;
    return this;
  }

  /**
   * Sets the drive motor constants for the drivetrain, and returns this DrivetrainConstants for
   * chaining.
   *
   * @param driveMotor The drive motor constants
   * @return This DrivetrainConstants object
   */
  public DrivetrainConstants withDriveMotor(DriveMotorConstants driveMotor) {
    DriveMotor = driveMotor;
    return this;
  }

  /**
   * Sets the steer motor constants for the drivetrain, and returns this DrivetrainConstants for
   * chaining.
   *
   * @param steerMotor The steer motor constants
   * @return This DrivetrainConstants object
   */
  public DrivetrainConstants withSteerMotor(SteerMotorConstants steerMotor) {
    SteerMotor = steerMotor;
    return this;
  }

  /**
   * Sets the steer encoder constants for the drivetrain, and returns this DrivetrainConstants for
   * chaining.
   *
   * @param steerEncoder The steer encoder constants
   * @return This DrivetrainConstants object
   */
  public DrivetrainConstants withSteerEncoder(SteerEncoderConstants steerEncoder) {
    SteerEncoder = steerEncoder;
    return this;
  }

  /**
   * Sets the swerve module constants for the drivetrain, and returns this DrivetrainConstants for
   * chaining.
   *
   * @param swerveModules The swerve module constants
   * @return This DrivetrainConstants object
   */
  public DrivetrainConstants withSwerveModules(SwerveModuleConstants[] swerveModules) {
    SwerveModules = swerveModules;
    return this;
  }

  /**
   * Sets the driving constants for the drivetrain, and returns this DrivetrainConstants for
   * chaining.
   *
   * @param driving The driving constants
   * @return This DrivetrainConstants object
   */
  public DrivetrainConstants withDriving(DrivingConstants driving) {
    Driving = driving;
    return this;
  }

  /**
   * Sets the simulation constants for the drivetrain, and returns this DrivetrainConstants for
   * chaining.
   *
   * @param simulation The simulation constants
   * @return This DrivetrainConstants object
   */
  public DrivetrainConstants withSimulation(SimulationConstants simulation) {
    Simulation = simulation;
    return this;
  }

  /**
   * Gets the swerve module constants for the specified module index.
   *
   * @param index The index of the swerve module
   * @return The swerve module constants
   */
  public SwerveModuleConstants getSwerveModule(int index) {
    return SwerveModules[index];
  }

  /**
   * Gets the swerve module constants for the specified module corner.
   *
   * @param corner The corner of the swerve drive
   * @return The swerve module constants
   */
  public SwerveModuleConstants getSwerveModule(Corner corner) {
    return SwerveModules[corner.getIndex()];
  }

  /**
   * Gets the wheel radius of the specified swerve module. If the module has a specific wheel radius
   * configured, that radius is returned. Otherwise, the default wheel radius from the
   * StructureConstants is returned.
   *
   * @param index The index of the swerve module
   * @return The wheel radius of the swerve module
   */
  public Distance getWheelRadius(int index) {
    SwerveModuleConstants module = SwerveModules[index];
    if (module.WheelRadius != null) {
      return module.WheelRadius;
    } else {
      return Structure.WheelRadius;
    }
  }

  /**
   * Gets the wheel radius of the specified swerve module. If the module has a specific wheel radius
   * configured, that radius is returned. Otherwise, the default wheel radius from the
   * StructureConstants is returned.
   *
   * @param corner The corner of the swerve drive
   * @return The wheel radius of the swerve module
   */
  public Distance getWheelRadius(Corner corner) {
    return getWheelRadius(corner.getIndex());
  }

  /**
   * Gets the steer gear reduction of the specified swerve module. If the module has a specific
   * steer gear reduction configured, that gear reduction is returned. Otherwise, the default steer
   * gear reduction from the SteerMotorConstants is returned.
   *
   * @param index The index of the swerve module
   * @return The steer gear reduction of the swerve module
   */
  public double getSteerGearReduction(int index) {
    SwerveModuleConstants module = SwerveModules[index];
    if (module.UniqueModuleConstants != null
        && module.UniqueModuleConstants.SteerGearReduction != 0) {
      return module.UniqueModuleConstants.SteerGearReduction;
    } else {
      return SteerMotor.GearReduction;
    }
  }

  /**
   * Gets the steer gear reduction of the specified swerve module. If the module has a specific
   * steer gear reduction configured, that gear reduction is returned. Otherwise, the default steer
   * gear reduction from the SteerMotorConstants is returned.
   *
   * @param corner The corner of the swerve drive
   * @return The steer gear reduction of the swerve module
   */
  public double getSteerGearReduction(Corner corner) {
    return getSteerGearReduction(corner.getIndex());
  }

  /**
   * Gets the steer motor configuration for the specified swerve module. If the module has a
   * specific steer motor configuration configured, that configuration is returned. Otherwise, the
   * default steer motor configuration from the SteerMotorConstants is returned.
   *
   * @param index The index of the swerve module
   * @return The steer motor configuration of the swerve module
   */
  public TalonFXConfiguration getSteerMotorConfig(int index) {
    SwerveModuleConstants module = SwerveModules[index];
    if (module.UniqueModuleConstants != null
        && module.UniqueModuleConstants.SteerMotorConfig != null) {
      return module.UniqueModuleConstants.SteerMotorConfig;
    } else {
      return SteerMotor.DeviceConfiguration;
    }
  }

  /**
   * Gets the steer motor configuration for the specified swerve module. If the module has a
   * specific steer motor configuration configured, that configuration is returned. Otherwise, the
   * default steer motor configuration from the SteerMotorConstants is returned.
   *
   * @param corner The corner of the swerve drive
   * @return The steer motor configuration of the swerve module
   */
  public TalonFXConfiguration getSteerMotorConfig(Corner corner) {
    return getSteerMotorConfig(corner.getIndex());
  }

  /**
   * Gets the simulated moment of inertia of the steer mechanism for the specified swerve module. If
   * the module has a specific simulated moment of inertia configured, that moment of inertia is
   * returned. Otherwise, the default simulated moment of inertia from the SteerMotorConstants is
   * returned.
   *
   * @param index The index of the swerve module
   * @return The simulated moment of inertia of the steer mechanism for the swerve module
   */
  public MomentOfInertia getSteerMomentOfInertia(int index) {
    SwerveModuleConstants module = SwerveModules[index];
    if (module.UniqueModuleConstants != null
        && module.UniqueModuleConstants.SteerMomentOfInertia != null) {
      return module.UniqueModuleConstants.SteerMomentOfInertia;
    } else {
      return SteerMotor.SimulatedMomentOfInertia;
    }
  }

  /**
   * Gets the simulated moment of inertia of the steer mechanism for the specified swerve module. If
   * the module has a specific simulated moment of inertia configured, that moment of inertia is
   * returned. Otherwise, the default simulated moment of inertia from the SteerMotorConstants is
   * returned.
   *
   * @param corner The corner of the swerve drive
   * @return The simulated moment of inertia of the steer mechanism for the swerve module
   */
  public MomentOfInertia getSteerMomentOfInertia(Corner corner) {
    return getSteerMomentOfInertia(corner.getIndex());
  }

  /**
   * Gets the drive motor configuration for the specified swerve module. If the module has a
   * specific drive motor configuration configured, that configuration is returned. Otherwise, the
   * default drive motor configuration from the DriveMotorConstants is returned.
   *
   * @param index The index of the swerve module
   * @return The drive motor configuration of the swerve module
   */
  public TalonFXConfiguration getDriveMotorConfig(int index) {
    SwerveModuleConstants module = SwerveModules[index];
    if (module.UniqueModuleConstants != null
        && module.UniqueModuleConstants.DriveMotorConfig != null) {
      return module.UniqueModuleConstants.DriveMotorConfig;
    } else {
      return DriveMotor.DeviceConfiguration;
    }
  }

  /**
   * Gets the drive motor configuration for the specified swerve module. If the module has a
   * specific drive motor configuration configured, that configuration is returned. Otherwise, the
   * default drive motor configuration from the DriveMotorConstants is returned.
   *
   * @param corner The corner of the swerve drive
   * @return The drive motor configuration of the swerve module
   */
  public TalonFXConfiguration getDriveMotorConfig(Corner corner) {
    return getDriveMotorConfig(corner.getIndex());
  }

  @Override
  public DrivetrainConstants clone() {
    try {
      return (DrivetrainConstants) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone should be supported", e);
    }
  }
}
