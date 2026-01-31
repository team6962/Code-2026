package com.team6962.lib.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.team6962.lib.swerve.MotionSwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class FuelLocalization {

  private static final double MAX_FOV_RATIO = Math.PI / 2;

  public static Translation2d getClumpPosition(
      PhotonCamera camera, MotionSwerveDrive swerveDrive, Translation3d cameraToRobot) {
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) return null;

    Translation2d bestPosition = null;
    double lowestTotalDistance = Double.MAX_VALUE;

    List<PhotonTrackedTarget> targets = result.getTargets();
    List<Translation2d> fuelLocations = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (target == null) continue;
      Translation2d fuelLocation = getFuelPosition(target, swerveDrive, cameraToRobot);
      if (fuelLocation == null) continue;
      fuelLocations.add(fuelLocation);
    }
    for (int i = 0; i < Math.min(fuelLocations.size(), FuelCameraConstants.MAX_TARGETS); i++) {
      double totalDistance = 0;
      Translation2d thisFuelPosition = fuelLocations.get(i);
      for (int j = 0; j < Math.min(fuelLocations.size(), FuelCameraConstants.MAX_TARGETS); j++) {
        if (i == j) continue;

        Translation2d otherFuelPosition = fuelLocations.get(j);
        if (otherFuelPosition != null) {
          double fuelDistance = thisFuelPosition.getDistance(otherFuelPosition);
          totalDistance = totalDistance + fuelDistance;
        }
      }
      if (totalDistance < lowestTotalDistance) {
        lowestTotalDistance = totalDistance;
        bestPosition = thisFuelPosition;
      }
    }
    return bestPosition;
  }

  public static Translation2d getFuelPosition(
      PhotonTrackedTarget target, MotionSwerveDrive swerveDrive, Translation3d cameraToRobot) {

    if (target == null || target.getDetectedObjectClassID() != 1)
      return null; // Check if detection is valid

    Translation2d fuelPosition = new Translation2d(0, 0);

    Angle horizontalOffset = Degrees.of(target.getYaw());

    double pixelHeight = getTargetHeight(target);
    double pixelWidth = getTargetWidth(target);
    double pixelRatio = pixelHeight / pixelWidth;

    if (!isValidSphericalObject(pixelHeight, pixelWidth, pixelRatio)) return null;

    double angularHeight =
        FuelCameraConstants.FOV_HEIGHT.getRadians()
            * (pixelHeight / FuelCameraConstants.FUEL_CAMERA_HEIGHT_PIXELS);
    if (angularHeight > MAX_FOV_RATIO) return null;

    Distance distance = calculateDistance(angularHeight);
    if (distance.gt(FuelCameraConstants.MAX_DETECTION_RANGE)) return null;

    Translation2d relativePosition =
        calculateRelativePosition(distance, horizontalOffset, cameraToRobot);

    Pose2d robotPosition = swerveDrive.getPosition2d();
    fuelPosition =
        robotPosition.getTranslation().plus(relativePosition.rotateBy(robotPosition.getRotation()));

    return fuelPosition;
  }

  // 0.0 is the NaN/Null value here
  private static double getTargetHeight(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners.size() < 4) return 0.0; // Ensure there are enough corners

    corners.sort((a, b) -> Double.compare(a.y, b.y));
    double avgMinY = (corners.get(0).y + corners.get(1).y) / 2.0;
    double avgMaxY = (corners.get(corners.size() - 1).y + corners.get(corners.size() - 2).y) / 2.0;

    double height = avgMaxY - avgMinY;
    return Double.isNaN(height) ? 0.0 : height; // Ensure NaN is handled
  }

  // 0.0 is the NaN/Null value here
  private static double getTargetWidth(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners.size() < 4) return 0.0; // Ensure there are enough corners

    corners.sort((a, b) -> Double.compare(a.x, b.x));
    double avgMinX = (corners.get(0).x + corners.get(1).x) / 2.0;
    double avgMaxX = (corners.get(corners.size() - 1).x + corners.get(corners.size() - 2).x) / 2.0;

    double width = avgMaxX - avgMinX;
    return Double.isNaN(width) ? 0.0 : width; // Ensure NaN is handled
  }

  private static boolean isValidSphericalObject(
      double pixelHeight, double pixelWidth, double pixelRatio) {
    if (pixelHeight <= 0 || pixelWidth <= 0) return false;
    return Math.abs(pixelRatio - 1.0) <= FuelCameraConstants.SPHERE_TOLERANCE;
  }

  private static Distance calculateDistance(double targetFOVRatio) {
    Distance absoluteDistance =
        Meters.of((FuelCameraConstants.FUEL_DIAMETER.in(Meters) / 2) / Math.tan(targetFOVRatio));
    return Meters.of(
        Math.sqrt(
            Math.pow(absoluteDistance.in(Meters), 2)
                - Math.pow(
                    Meters.of(FuelCameraConstants.FUEL_CAMERA_POSITION.getY()).in(Meters)
                        - (FuelCameraConstants.FUEL_DIAMETER.in(Meters)) / 2,
                    2)));
  }

  private static Translation2d calculateRelativePosition(
      Distance distance, Angle horizontalOffset, Translation3d cameraToRobot) {
    return new Translation2d(
            distance.in(Meters) * Math.cos(horizontalOffset.in(Radians)),
            -distance.in(Meters) * Math.sin(horizontalOffset.in(Radians)))
        .plus(cameraToRobot.toTranslation2d());
  }
}
