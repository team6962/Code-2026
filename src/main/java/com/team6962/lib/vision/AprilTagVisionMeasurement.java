package com.team6962.lib.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import org.photonvision.EstimatedRobotPose;

/**
 * Represents a single vision measurement from an AprilTag camera, including the estimated robot
 * pose and associated uncertainty (standard deviations). This class wraps PhotonVision's
 * EstimatedRobotPose with additional standard deviation information for sensor fusion with
 * odometry.
 */
public class AprilTagVisionMeasurement {
  /** The estimated robot pose from PhotonVision, including the 3D pose and timestamp. */
  private final EstimatedRobotPose photonEstimate;

  /**
   * Standard deviations for the pose estimate, used to weight the measurement in sensor fusion. The
   * matrix contains standard deviations for [x, y, z, rotation].
   */
  private final Matrix<N4, N1> stdDevs;

  /**
   * Constructs a vision measurement with the given pose estimate and standard deviations.
   *
   * @param photonEstimate The estimated robot pose from PhotonVision.
   * @param stdDevs The standard deviations for the pose estimate, used to weight this measurement
   *     in sensor fusion.
   */
  public AprilTagVisionMeasurement(EstimatedRobotPose photonEstimate, Matrix<N4, N1> stdDevs) {
    this.photonEstimate = photonEstimate;
    this.stdDevs = stdDevs;
  }

  /**
   * Gets the estimated 3D pose of the robot from this vision measurement.
   *
   * @return The estimated robot pose in 3D space.
   */
  public Pose3d getPose() {
    return photonEstimate.estimatedPose;
  }

  /**
   * Gets the timestamp when this vision measurement was captured.
   *
   * @return The timestamp in seconds.
   */
  public double getTimestamp() {
    return photonEstimate.timestampSeconds;
  }

  /**
   * Gets the underlying PhotonVision estimated robot pose object.
   *
   * @return The complete EstimatedRobotPose from PhotonVision.
   */
  public EstimatedRobotPose getPhotonEstimate() {
    return photonEstimate;
  }

  /**
   * Gets the standard deviations for this vision measurement, used to weight the measurement in
   * sensor fusion algorithms.
   *
   * @return A 4x1 matrix containing standard deviations for [x, y, z, rotation].
   */
  public Matrix<N4, N1> getStdDevs() {
    return stdDevs;
  }

  /**
   * Returns a copy of this vision measurement with the estimated orientation adjusted.
   *
   * @param newRotation The new rotation to set in the pose.
   * @return A new AprilTagVisionMeasurement with the updated rotation.
   */
  public AprilTagVisionMeasurement withAdjustedRotation(Rotation3d newRotation) {
    Pose3d adjustedPose = new Pose3d(photonEstimate.estimatedPose.getTranslation(), newRotation);

    EstimatedRobotPose adjustedEstimate =
        new EstimatedRobotPose(
            adjustedPose,
            photonEstimate.timestampSeconds,
            photonEstimate.targetsUsed,
            photonEstimate.strategy);

    return new AprilTagVisionMeasurement(adjustedEstimate, this.stdDevs);
  }

  /**
   * Creates a new AprilTagVisionMeasurement whose pose is expressed relative to the supplied
   * origin.
   *
   * @param origin the pose to which the returned measurement's pose will be made relative; this
   *     pose must be expressed in the same coordinate frame as {@code photonEstimate.estimatedPose}
   * @return a new {@code AprilTagVisionMeasurement} whose {@code EstimatedRobotPose} is relative to
   *     the given {@code origin}. Metadata (timestampSeconds, targetsUsed, strategy) and {@code
   *     stdDevs} are preserved from the original measurement.
   */
  public AprilTagVisionMeasurement relativeTo(Pose3d origin) {
    Pose3d relativePose = photonEstimate.estimatedPose.relativeTo(origin);

    EstimatedRobotPose relativeEstimate =
        new EstimatedRobotPose(
            relativePose,
            photonEstimate.timestampSeconds,
            photonEstimate.targetsUsed,
            photonEstimate.strategy);

    return new AprilTagVisionMeasurement(relativeEstimate, this.stdDevs);
  }
}
