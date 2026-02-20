package com.team6962.lib.vision;

import edu.wpi.first.math.geometry.Transform3d;

/** Configuration constants for an AprilTag camera. */
public class AprilTagCameraConstants implements Cloneable {
  /** The name of the camera. */
  public String Name;

  /** The transform from the robot to the camera. */
  public Transform3d Transform;

  /**
   * Constructs an AprilTagCameraConstants object with the specified parameters.
   *
   * @param name The name of the camera.
   * @param transform The transform from the robot to the camera.
   */
  public AprilTagCameraConstants(String name, Transform3d transform) {
    this.Name = name;
    this.Transform = transform;
  }

  /** Constructs an AprilTagCameraConstants object with default values. */
  public AprilTagCameraConstants() {
    this.Name = "";
    this.Transform = new Transform3d();
  }

  /**
   * Sets the name of the camera, and returns this AprilTagCameraConstants for chaining.
   *
   * @param name The name to set.
   * @return This AprilTagCameraConstants object.
   */
  public AprilTagCameraConstants withName(String name) {
    this.Name = name;
    return this;
  }

  /**
   * Sets the transform from the robot to the camera, and returns this AprilTagCameraConstants for
   * chaining.
   *
   * @param transform The transform to set.
   * @return This AprilTagCameraConstants object.
   */
  public AprilTagCameraConstants withTransform(Transform3d transform) {
    this.Transform = transform;
    return this;
  }
}
