package com.team6962.lib.swerve.localization;

import static edu.wpi.first.units.Units.Radians;

import com.team6962.lib.math.AngleMath;
import com.team6962.lib.swerve.gyro.Gyroscope;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;

/**
 * Fuses gyroscope, odometry, and vision data to estimate the robot's position
 * and velocity on the field.
 */
public class Localization {
    /**
     * The pose estimator that fuses gyroscope, odometry, and vision data to
     * estimate the robot's position on the field.
     */
    private SwerveDrivePoseEstimator poseEstimator;

    /**
     * The gyroscope used to obtain the robot's heading relative to its original
     * orientation.
     */
    private Gyroscope gyroscope;

    /**
     * The odometry object that provides the positions of the swerve modules.
     */
    private Odometry odometry;

    /**
     * The current field-relative velocity of the robot as a ChassisSpeeds
     * object.
     */
    private ChassisSpeeds velocity;

    /**
     * The most recent twist (change in position along a circular arc) computed
     * by the odometry.
     */
    private Twist2d twist;

    /**
     * The velocity along a circular arc, computed from odometry data. This is
     * stored in a Twist2d object, where dx, dy, and dtheta represent the
     * velocities in the x, y, and angular directions, respectively, instead of
     * changes in position.
     */
    private Twist2d arcVelocity;

    /**
     * The current yaw of the robot, stored as a continuous angle to prevent
     * discontinuities when crossing the ±180° boundary.
     */
    private Angle yaw = Radians.of(0);

    public Localization(SwerveDriveKinematics kinematics, Pose2d initialPose, Gyroscope gyroscope, Odometry odometry) {
        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            new Rotation2d(gyroscope.getYaw()),
            odometry.getPositions(),
            initialPose
        );

        this.gyroscope = gyroscope;
        this.odometry = odometry;
    }

    /**
     * Updates the localization estimates by fusing new gyroscope and odometry
     * data. This method should be called periodically to refresh the
     * localization data.
     * 
     * @param deltaTime The time elapsed since the last update, in seconds.
     */
    public void update(double deltaTime) {
        gyroscope.update();
        odometry.update();

        // Update the pose estimator with new gyroscope and odometry data
        poseEstimator.update(new Rotation2d(gyroscope.getYaw()), odometry.getPositions());

        yaw = AngleMath.toContinuous(poseEstimator.getEstimatedPosition().getRotation().getMeasure(), yaw);

        // Update the odometry to compute the latest twist
        twist = odometry.getTwist();

        // Compute the chassis velocity based on the twist and delta time
        velocity = ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(
            twist.dx / deltaTime,
            twist.dy / deltaTime,
            twist.dtheta / deltaTime
        ), poseEstimator.getEstimatedPosition().getRotation());

        // Compute the arc velocity as a Twist2d
        arcVelocity = new Twist2d(
            twist.dx / deltaTime,
            twist.dy / deltaTime,
            twist.dtheta / deltaTime
        );
    }

    /**
     * Gets the current estimated position of the robot on the field.
     * 
     * @return The current estimated Pose2d of the robot.
     */
    public Pose2d getPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the current heading of the robot, adjusted to be continuous with
     * the gyroscope reading.
     * 
     * @return
     */
    public Angle getHeading() {
        return AngleMath.toContinuous(poseEstimator.getEstimatedPosition().getRotation().getMeasure(), gyroscope.getYaw());
    }

    /**
     * Gets the field-relative velocity of the robot as a ChassisSpeeds object.
     * 
     * @return The current field-relative velocity of the robot.
     */
    public ChassisSpeeds getVelocity() {
        return velocity;
    }

    /**
     * Gets the robot-relative velocity along a circular arc, represented as a
     * Twist2d object.
     * 
     * @return The current arc velocity of the robot.
     */
    public Twist2d getArcVelocity() {
        return arcVelocity;
    }

    /**
     * Gets the most recent twist (change in position along a circular arc)
     * computed by the odometry.
     * 
     * @return The most recent Twist2d representing the change in position of
     * the robot.
     */
    public Twist2d getTwist() {
        return twist;
    }
}
