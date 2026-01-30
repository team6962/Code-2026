package com.team6962.lib.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class FuelCameraConstants {
  public String Name;

  /** The transform from the robot to the camera. */
  public Transform3d Transform;

  /**
   * Constructs an FuelCameraConstants object with the specified parameters.
   *
   * @param name The name of the camera.
   * @param transform The transform from the robot to the camera.
   */
  public FuelCameraConstants(String name, Transform3d transform) {
    this.Name = name;
    this.Transform = transform;
  }

  /** Constructs an FuelCameraConstants object with default values. */
  public FuelCameraConstants() {
    this.Name = "";
    this.Transform = new Transform3d();
  }

  /**
   * Sets the name of the camera, and returns this FuelCameraConstants for chaining.
   *
   * @param name The name to set.
   * @return This FuelCameraConstants object.
   */
  public FuelCameraConstants withName(String name) {
    this.Name = name;
    return this;
  }

  /**
   * Sets the transform from the robot to the camera, and returns this AprilTagCameraConstants for
   * chaining.
   *
   * @param transform The transform to set.
   * @return This FuelCameraConstants object.
   */
  public FuelCameraConstants withTransform(Transform3d transform) {
    this.Transform = transform;
    return this;
  }

  public static final Rotation2d FOV_HEIGHT = Rotation2d.fromDegrees(48.9);
  public static final Rotation2d FOV_WIDTH = Rotation2d.fromDegrees(70);
  public static final double FUEL_CAMERA_HEIGHT_PIXELS = 800;
  public static final Distance MAX_DETECTION_RANGE =
      Meters.of(18.37); // diagonal length of the field
  public static final double SPHERE_TOLERANCE = 0.5;
  public static final Distance FUEL_DIAMETER = Inches.of(5.91);

  public static final Translation3d FUEL_CAMERA_POSITION =
      new Translation3d(
          Units.inchesToMeters(14.0), -Units.inchesToMeters(1.0), Units.inchesToMeters(32.75));
}
