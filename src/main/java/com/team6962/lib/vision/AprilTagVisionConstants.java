package com.team6962.lib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import java.util.List;
import org.photonvision.simulation.SimCameraProperties;

/** Configuration constants for AprilTag vision processing. */
public class AprilTagVisionConstants implements Cloneable {
  /**
   * Standard deviations for single-tag pose estimates, which are generally less reliable than
   * multi-tag estimates.
   */
  public Matrix<N4, N1> SingleTagStdDevs;

  /**
   * Standard deviations for multi-tag pose estimates, which are generally more reliable than
   * single-tag estimates.
   */
  public Matrix<N4, N1> MultiTagStdDevs;

  /** Scalar to increase standard deviations based on distance to tags. */
  public double StdDevDistanceScalar;

  /**
   * The maximum distance at which a single tag detection is considered reliable enough to be used
   * for pose estimation.
   */
  public double MaxSingleTagDistance;

  /** The cameras used for AprilTag vision processing. */
  public List<AprilTagCameraConstants> Cameras;

  /** The AprilTag field layout used for pose estimation. */
  public AprilTagFieldLayout FieldLayout;

  /** The simulation camera properties. */
  public SimCameraProperties CameraSimProperties;

  /** Whether to draw wireframes for the simulated cameras. */
  public boolean DrawWireframes;

  /**
   * How many AprilTags must be detected to update the robot's heading using vision while enabled.
   */
  public int MinTagsForHeadingUpdateWhileEnabled;

  /**
   * How many AprilTags must be detected to update the robot's heading using vision while disabled.
   */
  public int MinTagsForHeadingUpdateWhileDisabled;

  /** Constructs an AprilTagVisionConstants object with default values. */
  public AprilTagVisionConstants() {
    SingleTagStdDevs = VecBuilder.fill(0.3, 0.3, 0.3, 0.6);
    MultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.1, 0.2);
    MaxSingleTagDistance = 4.0;
    StdDevDistanceScalar = 0.1;
    Cameras = List.of();
    FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    CameraSimProperties = new SimCameraProperties();
    DrawWireframes = false;
    MinTagsForHeadingUpdateWhileEnabled = 2;
    MinTagsForHeadingUpdateWhileDisabled = 1;
  }

  /**
   * Sets the standard deviations for single-tag pose estimates, and returns this
   * AprilTagVisionConstants for chaining.
   *
   * @param stdDevs The standard deviations matrix to set.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withSingleTagStdDevs(Matrix<N4, N1> stdDevs) {
    this.SingleTagStdDevs = stdDevs;
    return this;
  }

  /**
   * Sets the standard deviations for multi-tag pose estimates, and returns this
   * AprilTagVisionConstants for chaining.
   *
   * @param stdDevs The standard deviations matrix to set.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withMultiTagStdDevs(Matrix<N4, N1> stdDevs) {
    this.MultiTagStdDevs = stdDevs;
    return this;
  }

  /**
   * Sets the maximum distance for single tag detections to be considered reliable, and returns this
   * AprilTagVisionConstants for chaining.
   *
   * @param distance The maximum reliable distance in meters.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withMaxSingleTagDistance(double distance) {
    this.MaxSingleTagDistance = distance;
    return this;
  }

  /**
   * Sets the standard deviation distance scalar, and returns this AprilTagVisionConstants for
   * chaining.
   *
   * @param scalar The scalar value to set.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withStdDevDistanceScalar(double scalar) {
    this.StdDevDistanceScalar = scalar;
    return this;
  }

  /**
   * Sets the cameras used for AprilTag vision processing, and returns this AprilTagVisionConstants
   * for chaining.
   *
   * @param cameras The list of AprilTagCameraConstants to set.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withCameras(List<AprilTagCameraConstants> cameras) {
    this.Cameras = cameras;
    return this;
  }

  /**
   * Sets the cameras used for AprilTag vision processing, and returns this AprilTagVisionConstants
   * for chaining.
   *
   * @param cameras The AprilTagCameraConstants to set.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withCameras(AprilTagCameraConstants... cameras) {
    this.Cameras = List.of(cameras);
    return this;
  }

  /**
   * Sets the AprilTag field layout used for pose estimation, and returns this
   * AprilTagVisionConstants for chaining.
   *
   * @param fieldLayout The AprilTagFieldLayout to set.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withFieldLayout(AprilTagFieldLayout fieldLayout) {
    this.FieldLayout = fieldLayout;
    return this;
  }

  /**
   * Sets the simulation camera properties, and returns this AprilTagVisionConstants for chaining.
   *
   * @param properties The SimCameraProperties to set.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withCameraSimProperties(SimCameraProperties properties) {
    this.CameraSimProperties = properties;
    return this;
  }

  /**
   * Sets whether to draw wireframes for the simulated cameras, and returns this
   * AprilTagVisionConstants for chaining.
   *
   * @param drawWireframes True to draw wireframes, false otherwise.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withDrawWireframes(boolean drawWireframes) {
    this.DrawWireframes = drawWireframes;
    return this;
  }

  /**
   * Sets how many AprilTags must be detected to update the robot's heading using vision while it's
   * enabled, and returns this AprilTagVisionConstants for chaining.
   *
   * @param minTags The minimum number of tags required.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withMinTagsForHeadingUpdateWhileEnabled(int minTags) {
    this.MinTagsForHeadingUpdateWhileEnabled = minTags;
    return this;
  }

  /**
   * Sets how many AprilTags must be detected to update the robot's heading using vision while it's
   * disabled, and returns this AprilTagVisionConstants for chaining.
   *
   * @param minTags The minimum number of tags required.
   * @return This AprilTagVisionConstants object.
   */
  public AprilTagVisionConstants withMinTagsForHeadingUpdateWhileDisabled(int minTags) {
    this.MinTagsForHeadingUpdateWhileDisabled = minTags;
    return this;
  }

  @Override
  public AprilTagVisionConstants clone() {
    try {
      return (AprilTagVisionConstants) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone should be supported", e);
    }
  }
}
