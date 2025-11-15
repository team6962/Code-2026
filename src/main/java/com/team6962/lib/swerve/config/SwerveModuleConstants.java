package com.team6962.lib.swerve.config;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * The constants for each of the swerve modules in the drivetrain, which include
 * the CAN IDs of each device, absolute encoder offsets, and optional wheel
 * radii.
 */
public class SwerveModuleConstants {
    /**
     * The CAN ID of the drive motor for this module.
     */
    public int DriveMotorCANId = -1;

    /**
     * The CAN ID of the steer motor for this module.
     */
    public int SteerMotorCANId = -1;

    /**
     * The CAN ID of the steer absolute encoder for this module.
     */
    public int SteerEncoderCANId = -1;

    /**
     * The angular offset of the steer absolute encoder. This is the angle that
     * the wheel should be pointing when the module is facing forward relative
     * to the module. For the front left swerve module, this angle should be
     * where the wheel is is facing forwards relative to the robot, and for
     * other swerve modules it is rotated by the same amount as the module is
     * rotated.
     */
    public Angle SteerEncoderOffset = Radians.of(0);

    /**
     * The radius of the module's wheel. If this value is null, the wheel radius
     * from the swerve drive's configuration will be used instead.
     */
    public Distance WheelRadius = null;

    /**
     * The corner of a swerve drive that a module is located at.
     */
    public static enum Corner {
        FrontLeft(0, Degrees.of(0)),
        FrontRight(1, Degrees.of(-90)),
        BackLeft(2, Degrees.of(90)),
        BackRight(3, Degrees.of(180));

        private final int index;
        private final Angle rotation;

        private Corner(int index, Angle rotation) {
            this.index = index;
            this.rotation = rotation;
        }

        /**
         * Gets the index of this corner, which is a number from 0 to 3:
         * <p>
         * 0 = Front Left, 1 = Front Right, 2 = Back Left, 3 = Back Right
         * <p>
         * This is the order used for arrays of swerve modules or their states
         * when performing kinematics calculations.
         * 
         * @return The index of this corner
         */
        public int getIndex() {
            return index;
        }

        /**
         * Gets the rotation of this corner relative to the robot's forward
         * direction.
         * 
         * @return The rotation of this corner
         */
        public Angle getRotation() {
            return rotation;
        }
    }
}
