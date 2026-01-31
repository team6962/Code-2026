package com.team6962.lib.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class FuelCameraConstants {
  public static final String NAME = "FuelCamera";

  public static final Rotation2d FOV_HEIGHT = Rotation2d.fromDegrees(48.9);
  public static final Rotation2d FOV_WIDTH = Rotation2d.fromDegrees(70);
  public static final double FUEL_CAMERA_HEIGHT_PIXELS = 800;
  public static final Distance MAX_DETECTION_RANGE =
      Meters.of(18.37); // diagonal length of the field
  public static final double SPHERE_TOLERANCE = 0.25;
  public static final Distance FUEL_DIAMETER = Inches.of(5.91);
  public static final int MAX_TARGETS = 50;
  public static final Translation3d FUEL_CAMERA_POSITION =
      new Translation3d(
          Units.inchesToMeters(14.0), -Units.inchesToMeters(1.0), Units.inchesToMeters(32.75));
}
