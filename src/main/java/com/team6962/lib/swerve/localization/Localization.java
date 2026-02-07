package com.team6962.lib.swerve.localization;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.math.AngleMath;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.util.SwerveComponent;
import com.team6962.lib.vision.AprilTagVisionMeasurement;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.Angle;

/**
 * Fuses gyroscope, odometry, and vision data to estimate the robot's position and velocity on the
 * field.
 */
public class Localization implements SwerveComponent {
  /**
   * The pose estimator that fuses gyroscope, odometry, and vision data to estimate the robot's
   * position on the field.
   */
  private SwerveDrivePoseEstimator3d poseEstimator;

  /** The gyroscope used to obtain the robot's heading relative to its original orientation. */
  private Gyroscope gyroscope;

  /** The odometry object that provides the positions of the swerve modules. */
  private Odometry odometry;

  /** The current field-relative velocity of the robot as a ChassisSpeeds object. */
  private ChassisSpeeds velocity;

  /**
   * The current field-relative translation velocity of the robot as a TranslationalVelocity object.
   */
  private TranslationalVelocity translationalVelocity;

  /** The most recent twist (change in position along a circular arc) computed by the odometry. */
  private Twist2d twist;

  /**
   * The velocity along a circular arc, computed from odometry data. This is stored in a Twist2d
   * object, where dx, dy, and dtheta represent the velocities in the x, y, and angular directions,
   * respectively, instead of changes in position.
   */
  private Twist2d arcVelocity;

  /**
   * The current yaw of the robot, stored as a continuous angle to prevent discontinuities when
   * crossing the ±180° boundary.
   */
  private Angle yaw = Radians.of(0);

  public Localization(
      DrivetrainConstants constants, Pose3d initialPose, Odometry odometry, Gyroscope gyroscope) {
    this.gyroscope = gyroscope;
    this.odometry = odometry;

    this.poseEstimator =
        new SwerveDrivePoseEstimator3d(
            constants.Structure.getKinematics(),
            gyroscope.getRotation(),
            odometry.getPositions(),
            initialPose);
  }

  @Override
  public synchronized void update(double deltaTimeSeconds) {
    // Update the pose estimator with new gyroscope and odometry data
    poseEstimator.update(gyroscope.getRotation(), odometry.getPositions());

    yaw =
        AngleMath.toContinuous(
            poseEstimator.getEstimatedPosition().getRotation().getMeasureZ(), yaw);

    // Update the odometry to compute the latest twist
    twist = odometry.getTwist();

    // Compute the chassis velocity based on the twist and delta time
    velocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            new ChassisSpeeds(
                twist.dx / deltaTimeSeconds,
                twist.dy / deltaTimeSeconds,
                twist.dtheta / deltaTimeSeconds),
            new Rotation2d(poseEstimator.getEstimatedPosition().getRotation().getMeasureZ()));

    // Update the translational velocity
    translationalVelocity =
        new TranslationalVelocity(
            MetersPerSecond.of(velocity.vxMetersPerSecond),
            MetersPerSecond.of(velocity.vyMetersPerSecond));

    // Compute the arc velocity as a Twist2d
    arcVelocity =
        new Twist2d(
            twist.dx / deltaTimeSeconds,
            twist.dy / deltaTimeSeconds,
            twist.dtheta / deltaTimeSeconds);
  }

  @Override
  public BaseStatusSignal[] getStatusSignals() {
    return SwerveComponent.combineStatusSignals(gyroscope, odometry);
  }

  @Override
  public ParentDevice[] getPhoenixDevices() {
    return SwerveComponent.combinePhoenixDevices(gyroscope, odometry);
  }

  @Override
  public synchronized void logTelemetry(String basePath) {
    basePath = LoggingUtil.ensureEndsWithSlash(basePath);

    gyroscope.logTelemetry(basePath + "Gyroscope");
    odometry.logTelemetry(basePath + "Odometry");

    Pose3d position = getPosition3d();

    DogLog.log(basePath + "Localization/Position", position);

    DogLog.log(basePath + "Localization/PositionX", position.getX(), Meters);
    DogLog.log(basePath + "Localization/PositionY", position.getY(), Meters);
    DogLog.log(basePath + "Localization/PositionZ", position.getZ(), Meters);

    DogLog.log(basePath + "Localization/RotationYaw", position.getRotation().getZ(), Radians);
    DogLog.log(basePath + "Localization/RotationPitch", position.getRotation().getY(), Radians);
    DogLog.log(basePath + "Localization/RotationRoll", position.getRotation().getX(), Radians);

    DogLog.log(basePath + "Localization/Heading", getHeading().in(Radians), Radians);

    DogLog.log(basePath + "Localization/VelocityX", velocity.vxMetersPerSecond, MetersPerSecond);
    DogLog.log(basePath + "Localization/VelocityY", velocity.vyMetersPerSecond, MetersPerSecond);
    DogLog.log(
        basePath + "Localization/AngularVelocity",
        velocity.omegaRadiansPerSecond,
        RadiansPerSecond);

    DogLog.log(
        basePath + "Localization/AngularVelocityYaw",
        gyroscope.getYawVelocity().in(RadiansPerSecond),
        RadiansPerSecond);
    DogLog.log(
        basePath + "Localization/AngularVelocityPitch",
        gyroscope.getPitchVelocity().in(RadiansPerSecond),
        RadiansPerSecond);
    DogLog.log(
        basePath + "Localization/AngularVelocityRoll",
        gyroscope.getRollVelocity().in(RadiansPerSecond),
        RadiansPerSecond);

    DogLog.log(basePath + "Localization/TwistDX", twist.dx, Meters);
    DogLog.log(basePath + "Localization/TwistDY", twist.dy, Meters);
    DogLog.log(basePath + "Localization/TwistDTheta", twist.dtheta, Radians);

    DogLog.log(basePath + "Localization/ArcVelocityDX", arcVelocity.dx, MetersPerSecond);
    DogLog.log(basePath + "Localization/ArcVelocityDY", arcVelocity.dy, MetersPerSecond);
    DogLog.log(basePath + "Localization/ArcVelocityDTheta", arcVelocity.dtheta, RadiansPerSecond);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * @param pose The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that you must
   *     use a timestamp with an epoch since FPGA startup. This means that you should use {@link
   *     edu.wpi.first.wpilibj.Timer#getFPGATimestamp() Timer.getFPGATimestamp()} as your time
   *     source.
   * @param stdDevs Standard deviations of the vision pose measurement (x position in meters, y
   *     position in meters, z position in meters, and angle in radians). Increase these numbers to
   *     trust the vision pose measurement less.
   */
  public synchronized void addVisionMeasurement(
      Pose3d pose, double timestampSeconds, Matrix<N4, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(pose, timestampSeconds, stdDevs);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * @param pose The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that you must
   *     use a timestamp with an epoch since FPGA startup. This means that you should use {@link
   *     edu.wpi.first.wpilibj.Timer#getFPGATimestamp() Timer.getFPGATimestamp()} as your time
   *     source.
   * @param stdDevs Standard deviations of the vision pose measurement (x position in meters, y
   *     position in meters, and angle in radians). Increase these numbers to trust the vision pose
   *     measurement less.
   */
  public synchronized void addVisionMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    Vector<N4> stdDevs3d = new Vector<N4>(Nat.N4());

    stdDevs3d.set(0, 0, stdDevs.get(0, 0));
    stdDevs3d.set(1, 0, stdDevs.get(1, 0));
    stdDevs3d.set(2, 0, 0.0);
    stdDevs3d.set(3, 0, stdDevs.get(2, 0));

    poseEstimator.addVisionMeasurement(new Pose3d(pose), timestampSeconds, stdDevs3d);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param pose The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that you must
   *     use a timestamp with an epoch since FPGA startup. This means that you should use {@link
   *     edu.wpi.first.wpilibj.Timer#getFPGATimestamp() Timer.getFPGATimestamp()} as your time
   *     source.
   */
  public synchronized void addVisionMeasurement(Pose3d pose, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(pose, timestampSeconds);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param pose The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that you must
   *     use a timestamp with an epoch since FPGA startup. This means that you should use {@link
   *     edu.wpi.first.wpilibj.Timer#getFPGATimestamp() Timer.getFPGATimestamp()} as your time
   *     source.
   */
  public synchronized void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(new Pose3d(pose), timestampSeconds);
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param measurement The vision measurement containing the pose, timestamp, and standard
   *     deviations.
   */
  public synchronized void addVisionMeasurement(AprilTagVisionMeasurement measurement) {
    addVisionMeasurement(
        measurement.getPose(), measurement.getTimestamp(), measurement.getStdDevs());
  }

  /**
   * Sets the current estimated position of the robot on the field.
   *
   * @param pose The new estimated Pose3d of the robot.
   */
  public synchronized void resetPosition(Pose3d pose) {
    poseEstimator.resetPose(pose);
  }

  /**
   * Sets the current estimated position of the robot on the field.
   *
   * @param pose The new estimated Pose2d of the robot.
   */
  public synchronized void resetPosition(Pose2d pose) {
    poseEstimator.resetPose(new Pose3d(pose));
  }

  /**
   * Sets the estimated yaw of the robot to the specified angle.
   *
   * @param yaw The new yaw angle for the robot.
   */
  public synchronized void resetYaw(Angle yaw) {
    Pose3d currentPose = poseEstimator.getEstimatedPosition();
    Pose3d newPose =
        new Pose3d(
            currentPose.getTranslation(),
            new Rotation3d(
                currentPose.getRotation().getX(),
                currentPose.getRotation().getY(),
                yaw.in(Radians)));

    poseEstimator.resetPose(newPose);
    this.yaw = yaw;
  }

  /** Sets the estimated yaw of the robot to zero (forwards). */
  public synchronized void resetYaw() {
    resetYaw(Radians.of(0));
  }

  /**
   * Gets the current estimated position of the robot on the field.
   *
   * @return The current estimated Pose2d of the robot.
   */
  public synchronized Pose2d getPosition2d() {
    return poseEstimator.getEstimatedPosition().toPose2d();
  }

  /**
   * Gets the current estimated position of the robot on the field.
   *
   * @return The current estimated Pose3d of the robot.
   */
  public synchronized Pose3d getPosition3d() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the current heading of the robot, adjusted to be continuous with the gyroscope reading.
   *
   * @return
   */
  public synchronized Angle getHeading() {
    return AngleMath.toContinuous(
        poseEstimator.getEstimatedPosition().getRotation().getMeasureZ(), gyroscope.getYaw());
  }

  /**
   * Gets the current rotation of the robot as a Rotation3d object.
   *
   * @return The current rotation of the robot.
   */
  public synchronized Rotation3d getRotation3d() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Gets the pitch of the robot.
   *
   * @return The current pitch of the robot.
   */
  public synchronized Angle getPitch() {
    return poseEstimator.getEstimatedPosition().getRotation().getMeasureX();
  }

  /**
   * Gets the roll of the robot.
   *
   * @return The current roll of the robot.
   */
  public synchronized Angle getRoll() {
    return poseEstimator.getEstimatedPosition().getRotation().getMeasureY();
  }

  /**
   * Gets the yaw of the robot.
   *
   * @return The current yaw of the robot.
   */
  public synchronized Angle getYaw() {
    return getHeading();
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
   * Gets the field-relative translational velocity of the robot as a TranslationalVelocity object.
   *
   * @return The current field-relative translational velocity of the robot.
   */
  public synchronized TranslationalVelocity getTranslationalVelocity() {
    return translationalVelocity;
  }

  /**
   * Gets the robot-relative velocity along a circular arc, represented as a Twist2d object.
   *
   * @return The current arc velocity of the robot.
   */
  public synchronized Twist2d getArcVelocity() {
    return arcVelocity;
  }

  /**
   * Gets the most recent twist (change in position along a circular arc) computed by the odometry.
   *
   * @return The most recent Twist2d representing the change in position of the robot.
   */
  public synchronized Twist2d getTwist() {
    return twist;
  }
}
