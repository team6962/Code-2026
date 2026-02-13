package com.team6962.lib.swerve.config;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * The constants for each of the swerve modules in the drivetrain, which include the CAN IDs of each
 * device, absolute encoder offsets, and optional wheel radii.
 */
public class SwerveModuleConstants implements Cloneable {
  /** The CAN ID of the drive motor for this module. */
  public int DriveMotorCANId = -1;

  /** The CAN ID of the steer motor for this module. */
  public int SteerMotorCANId = -1;

  /** The CAN ID of the steer absolute encoder for this module. */
  public int SteerEncoderCANId = -1;

  /**
   * The angular offset of the steer absolute encoder. This is the angle that the wheel should be
   * pointing when the module is facing forward relative to the module. For the front left swerve
   * module, this angle should be where the wheel is is facing forwards relative to the robot, and
   * for other swerve modules it is rotated by the same amount as the module is rotated.
   */
  public Angle SteerEncoderOffset = Radians.of(0);

  /**
   * The radius of the module's wheel. If this value is null, the wheel radius from the swerve
   * drive's configuration will be used instead.
   */
  public Distance WheelRadius = null;

  /**
   * Optional constants that are unique to this module, such as a different motor configuration or
   * different PID constants. If this is null, the module constants from the swerve drive's
   * configuration will be used instead.
   */
  public UniqueModuleConstants UniqueModuleConstants = null;

  /** Constructs a SwerveModuleConstants object with default values. */
  public SwerveModuleConstants() {}

  /**
   * Sets the CAN ID of the drive motor, and returns this SwerveModuleConstants for chaining.
   *
   * @param driveMotorCANId The CAN ID of the drive motor
   * @return This SwerveModuleConstants object
   */
  public SwerveModuleConstants withDriveMotorCANId(int driveMotorCANId) {
    DriveMotorCANId = driveMotorCANId;
    return this;
  }

  /**
   * Sets the CAN ID of the steer motor, and returns this SwerveModuleConstants for chaining.
   *
   * @param steerMotorCANId The CAN ID of the steer motor
   * @return This SwerveModuleConstants object
   */
  public SwerveModuleConstants withSteerMotorCANId(int steerMotorCANId) {
    SteerMotorCANId = steerMotorCANId;
    return this;
  }

  /**
   * Sets the CAN ID of the steer encoder, and returns this SwerveModuleConstants for chaining.
   *
   * @param steerEncoderCANId The CAN ID of the steer encoder
   * @return This SwerveModuleConstants object
   */
  public SwerveModuleConstants withSteerEncoderCANId(int steerEncoderCANId) {
    SteerEncoderCANId = steerEncoderCANId;
    return this;
  }

  /**
   * Sets the steer encoder offset, and returns this SwerveModuleConstants for chaining.
   *
   * @param steerEncoderOffset The steer encoder offset
   * @return This SwerveModuleConstants object
   */
  public SwerveModuleConstants withSteerEncoderOffset(Angle steerEncoderOffset) {
    SteerEncoderOffset = steerEncoderOffset;
    return this;
  }

  /**
   * Sets the wheel radius, and returns this SwerveModuleConstants for chaining.
   *
   * @param wheelRadius The wheel radius
   * @return This SwerveModuleConstants object
   */
  public SwerveModuleConstants withWheelRadius(Distance wheelRadius) {
    WheelRadius = wheelRadius;
    return this;
  }

  /**
   * Sets the unique module constants, and returns this SwerveModuleConstants for chaining.
   *
   * @param uniqueModuleConstants The unique module constants
   * @return This SwerveModuleConstants object
   */
  public SwerveModuleConstants withUniqueModuleConstants(
      UniqueModuleConstants uniqueModuleConstants) {
    UniqueModuleConstants = uniqueModuleConstants;
    return this;
  }

  /** The corner of a swerve drive that a module is located at. */
  public static enum Corner {
    FrontLeft(0, Degrees.of(0), "Front Left"),
    FrontRight(1, Degrees.of(-90), "Front Right"),
    BackLeft(2, Degrees.of(90), "Back Left"),
    BackRight(3, Degrees.of(180), "Back Right");

    private final int index;
    private final Angle rotation;
    private final String name;

    private Corner(int index, Angle rotation, String name) {
      this.index = index;
      this.rotation = rotation;
      this.name = name;
    }

    /**
     * Gets the index of this corner, which is a number from 0 to 3:
     *
     * <p>0 = Front Left, 1 = Front Right, 2 = Back Left, 3 = Back Right
     *
     * <p>This is the order used for arrays of swerve modules or their states when performing
     * kinematics calculations.
     *
     * @return The index of this corner
     */
    public int getIndex() {
      return index;
    }

    /**
     * Gets the rotation of this corner relative to the robot's forward direction.
     *
     * @return The rotation of this corner
     */
    public Angle getRotation() {
      return rotation;
    }

    /**
     * Gets the name of this corner.
     *
     * @return The name of this corner
     */
    public String getName() {
      return name;
    }

    @Override
    public String toString() {
      return name;
    }

    /** Gets the Corner enum value for a given index. */
    public static Corner fromIndex(int index) {
      for (Corner corner : Corner.values()) {
        if (corner.index == index) {
          return corner;
        }
      }

      throw new IllegalArgumentException("Invalid corner index: " + index);
    }
  }

  @Override
  public SwerveModuleConstants clone() {
    try {
      return (SwerveModuleConstants) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone should be supported", e);
    }
  }
}
