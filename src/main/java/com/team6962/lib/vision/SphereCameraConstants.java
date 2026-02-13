package com.team6962.lib.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

/**
 * Configuration constants for a sphere detection camera. This class defines camera properties,
 * detection parameters, and physical mounting position for a camera used to detect and locate
 * sphere game pieces on the field.
 */
public class SphereCameraConstants implements Cloneable {
  /** The name identifier for this sphere detection camera. */
  public String Name = "SphereCamera";

  /** The class id of the spheres. */
  public int ClassId = 1;

  /** The vertical field of view of the camera. */
  public Rotation2d FOVHeight = Rotation2d.fromDegrees(48.9);

  /** The horizontal field of view of the camera. */
  public Rotation2d FOVWidth = Rotation2d.fromDegrees(70);

  /** The height of the camera image in pixels. */
  public double CameraHeightPixels = 800;

  /** The height of the camera image in pixels. */
  public double CameraWidthPixels = 1280;

  /** The focal length of the camera in the X direction */
  public double CameraFocalLengthX = 907.41;

  /** The focal length of the camera in the Y direction */
  public double CameraFocalLengthY = 907.64;

  /**
   * Maximum range at which sphere pieces can be detected. Default is the diagonal length of the
   * field.
   */
  public Distance MaxDetectionRange = Meters.of(18.37);

  /**
   * Tolerance for sphere detection algorithm. This affects how closely a detected object must match
   * a spherical shape to be considered a sphere piece.
   */
  public double SphereTolerance = 0.1;

  /** The diameter of a sphere game piece. */
  public Distance SphereDiameter = Inches.of(5.91);

  /** Maximum number of targets that can be tracked simultaneously. */
  public int MaxTargets = 50;

  /** The 3D transform of the sphere detection camera relative to the robot's center. */
  public Transform3d RobotToCameraTransform = new Transform3d();

  /** Constructs a SphereCameraConstants with default values. */
  public SphereCameraConstants() {}

  /**
   * Constructs a SphereCameraConstants with the specified camera name. Other fields need to be set
   * via the 'with' methods or by assigning the fields before these constants can be used.
   *
   * @param name The name identifier for this sphere camera.
   */
  public SphereCameraConstants(String name) {
    this.Name = name;
  }

  /**
   * Sets the name of the sphere camera.
   *
   * @param name The name identifier for this sphere camera.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withName(String name) {
    this.Name = name;
    return this;
  }

  /**
   * Sets the class id of the spheres.
   *
   * @param classId The class id of the spheres.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withClassId(int classId) {
    this.ClassId = classId;
    return this;
  }

  /**
   * Sets the vertical field of view of the camera.
   *
   * @param fovHeight The vertical field of view.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withFOVHeight(Rotation2d fovHeight) {
    this.FOVHeight = fovHeight;
    return this;
  }

  /**
   * Sets the horizontal field of view of the camera.
   *
   * @param fovWidth The horizontal field of view.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withFOVWidth(Rotation2d fovWidth) {
    this.FOVWidth = fovWidth;
    return this;
  }

  /**
   * Sets the height of the camera image in pixels.
   *
   * @param heightPixels The height of the camera image in pixels.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withCameraHeightPixels(double heightPixels) {
    this.CameraHeightPixels = heightPixels;
    return this;
  }

  /**
   * Sets the width of the camera image in pixels.
   *
   * @param widthPixels The width of the camera image in pixels.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withCameraWidthPixels(double widthPixels) {
    this.CameraWidthPixels = widthPixels;
    return this;
  }

  /**
   * Sets focal lenght of the camera in the X direction in pixels.
   *
   * @param FocalLengthX The focal lenght of the camera in the X direction in pixels.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withFocalLengthX(double focalLengthX) {
    this.CameraFocalLengthX = focalLengthX;
    return this;
  }

  /**
   * Sets focal lenght of the camera in the Y direction in pixels.
   *
   * @param FocalLengthY The focal lenght of the camera in the Y direction in pixels.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withFocalLengthY(double focalLengthY) {
    this.CameraFocalLengthY = focalLengthY;
    return this;
  }

  /**
   * Sets the maximum range at which sphere pieces can be detected.
   *
   * @param maxRange The maximum detection range.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withMaxDetectionRange(Distance maxRange) {
    this.MaxDetectionRange = maxRange;
    return this;
  }

  /**
   * Sets the tolerance for sphere detection algorithm.
   *
   * @param tolerance The sphere detection tolerance.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withSphereTolerance(double tolerance) {
    this.SphereTolerance = tolerance;
    return this;
  }

  /**
   * Sets the diameter of the sphere game piece.
   *
   * @param diameter The sphere piece diameter.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withSphereDiameter(Distance diameter) {
    this.SphereDiameter = diameter;
    return this;
  }

  /**
   * Sets the maximum number of targets that can be tracked simultaneously.
   *
   * @param maxTargets The maximum number of targets.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withMaxTargets(int maxTargets) {
    this.MaxTargets = maxTargets;
    return this;
  }

  /**
   * Sets the robot-to-camera transform.
   *
   * @param transform The camera transform.
   * @return This SphereCameraConstants instance for method chaining.
   */
  public SphereCameraConstants withRobotToCameraTransform(Transform3d transform) {
    this.RobotToCameraTransform = transform;
    return this;
  }

  @Override
  public SphereCameraConstants clone() {
    try {
      return (SphereCameraConstants) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone should be supported", e);
    }
  }
}
