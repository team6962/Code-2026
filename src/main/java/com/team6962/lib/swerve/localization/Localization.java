package com.team6962.lib.swerve.localization;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.math.AngleMath;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.util.SwerveComponent;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;

/**
 * Fuses gyroscope, odometry, and vision data to estimate the robot's position
 * and velocity on the field.
 */
public class Localization implements SwerveComponent {
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
     * The current field-relative translation velocity of the robot as a
     * TranslationalVelocity object.
     */
    private TranslationalVelocity translationalVelocity;

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

    public Localization(DrivetrainConstants constants, Pose2d initialPose, Odometry odometry, Gyroscope gyroscope) {
        this.gyroscope = gyroscope;
        this.odometry = odometry;
        
        this.poseEstimator = new SwerveDrivePoseEstimator(
            constants.Structure.getKinematics(),
            new Rotation2d(gyroscope.getYaw()),
            odometry.getPositions(),
            initialPose
        );
    }

    @Override
    public synchronized void update(double deltaTimeSeconds) {
        // Update the pose estimator with new gyroscope and odometry data
        poseEstimator.update(new Rotation2d(gyroscope.getYaw()), odometry.getPositions());

        yaw = AngleMath.toContinuous(poseEstimator.getEstimatedPosition().getRotation().getMeasure(), yaw);

        // Update the odometry to compute the latest twist
        twist = odometry.getTwist();

        // Compute the chassis velocity based on the twist and delta time
        velocity = ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(
            twist.dx / deltaTimeSeconds,
            twist.dy / deltaTimeSeconds,
            twist.dtheta / deltaTimeSeconds
        ), poseEstimator.getEstimatedPosition().getRotation());

        // Update the translational velocity
        translationalVelocity = new TranslationalVelocity(
            MetersPerSecond.of(velocity.vxMetersPerSecond),
            MetersPerSecond.of(velocity.vyMetersPerSecond)
        );

        // Compute the arc velocity as a Twist2d
        arcVelocity = new Twist2d(
            twist.dx / deltaTimeSeconds,
            twist.dy / deltaTimeSeconds,
            twist.dtheta / deltaTimeSeconds
        );
    }

    @Override
    public BaseStatusSignal[] getStatusSignals() {
        return SwerveComponent.combineStatusSignals(gyroscope, odometry);
    }

    @Override
    public synchronized void logTelemetry(String basePath) {
        basePath = LoggingUtil.ensureEndsWithSlash(basePath);

        gyroscope.logTelemetry(basePath + "/Gyroscope");
        odometry.logTelemetry(basePath + "/Odometry");

        Pose2d position = getPosition();

        DogLog.log(basePath + "Localization/PositionX", position.getX(), Meters);
        DogLog.log(basePath + "Localization/PositionY", position.getY(), Meters);
        DogLog.log(basePath + "Localization/Heading", getHeading().in(Radians), Radians);

        DogLog.log(basePath + "Localization/VelocityX", velocity.vxMetersPerSecond, MetersPerSecond);
        DogLog.log(basePath + "Localization/VelocityY", velocity.vyMetersPerSecond, MetersPerSecond);
        DogLog.log(basePath + "Localization/AngularVelocity", velocity.omegaRadiansPerSecond, RadiansPerSecond);

        DogLog.log(basePath + "Localization/TwistDX", twist.dx, Meters);
        DogLog.log(basePath + "Localization/TwistDY", twist.dy, Meters);
        DogLog.log(basePath + "Localization/TwistDTheta", twist.dtheta, Radians);

        DogLog.log(basePath + "Localization/ArcVelocityDX", arcVelocity.dx, MetersPerSecond);
        DogLog.log(basePath + "Localization/ArcVelocityDY", arcVelocity.dy, MetersPerSecond);
        DogLog.log(basePath + "Localization/ArcVelocityDTheta", arcVelocity.dtheta, RadiansPerSecond);
    }

    public synchronized void addVisionEstimate(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ) {
        poseEstimator.addVisionMeasurement(pose, timestampSeconds, stdDevs);
    }

    public synchronized void addVisionEstimate(
        Pose2d pose,
        double timestampSeconds
    ) {
        poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    /**
     * Gets the current estimated position of the robot on the field.
     * 
     * @return The current estimated Pose2d of the robot.
     */
    public synchronized Pose2d getPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the current heading of the robot, adjusted to be continuous with
     * the gyroscope reading.
     * 
     * @return
     */
    public synchronized Angle getHeading() {
        return AngleMath.toContinuous(poseEstimator.getEstimatedPosition().getRotation().getMeasure(), gyroscope.getYaw());
    }

    /**
     * Gets the field-relative velocity of the robot as a ChassisSpeeds object.
     * 
     * @return The current field-relative velocity of the robot.
     */
    public synchronized ChassisSpeeds getVelocity() {
        return velocity;
    }

    /**
     * Gets the field-relative translational velocity of the robot as a
     * TranslationalVelocity object.
     * 
     * @return The current field-relative translational velocity of the robot.
     */
    public synchronized TranslationalVelocity getTranslationalVelocity() {
        return translationalVelocity;
    }

    /**
     * Gets the robot-relative velocity along a circular arc, represented as a
     * Twist2d object.
     * 
     * @return The current arc velocity of the robot.
     */
    public synchronized Twist2d getArcVelocity() {
        return arcVelocity;
    }

    /**
     * Gets the most recent twist (change in position along a circular arc)
     * computed by the odometry.
     * 
     * @return The most recent Twist2d representing the change in position of
     * the robot.
     */
    public synchronized Twist2d getTwist() {
        return twist;
    }
}
