package com.team6962.lib.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.team6962.lib.swerve.MotionSwerveDrive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class SphereClumpLocalization {
  private MotionSwerveDrive swerveDrive;
  private PhotonCamera camera;
  private SphereCameraConstants cameraConstants;

  public SphereClumpLocalization(MotionSwerveDrive swerveDrive, SphereCameraConstants cameraConstants) {
    this.swerveDrive = swerveDrive;
    this.camera = new PhotonCamera(cameraConstants.Name);
    this.cameraConstants = cameraConstants;
  }

  public Translation2d getClumpPosition() {
    // TODO: Switch to getAllUnreadResults()
    @SuppressWarnings("removal")
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) return null;

    Translation2d bestPosition = null;
    double lowestTotalDistance = Double.MAX_VALUE;

    List<PhotonTrackedTarget> targets = result.getTargets();
    List<Translation2d> fuelLocations = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      if (target == null) continue;
      Translation2d fuelLocation = getFuelPosition(target);
      if (fuelLocation == null) continue;
      fuelLocations.add(fuelLocation);
    }
    for (int i = 0; i < Math.min(fuelLocations.size(), cameraConstants.MaxTargets); i++) {
      double totalDistance = 0;
      Translation2d thisFuelPosition = fuelLocations.get(i);
      for (int j = 0; j < Math.min(fuelLocations.size(), cameraConstants.MaxTargets); j++) {
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

  public Translation2d getFuelPosition(PhotonTrackedTarget target) {
    if (target == null || target.getDetectedObjectClassID() != 1)
      return null; // Check if detection is valid

    Angle yaw = Degrees.of(target.getYaw());
    Angle pitch = Degrees.of(target.getPitch());

    double pixelHeight = getTargetHeight(target);
    double pixelWidth = getTargetWidth(target);
    double pixelRatio = pixelHeight / pixelWidth;

    if (!isValidSphericalObject(pixelHeight, pixelWidth, pixelRatio)) return null;

    double angularHeight =
        cameraConstants.FOVHeight.getRadians()
            * (pixelHeight / cameraConstants.CameraHeightPixels);
    if (angularHeight > Math.PI / 2) return null;

    Distance distance = calculateDistance(angularHeight);
    if (distance.gt(cameraConstants.MaxDetectionRange)) return null;

    Transform3d cameraRelativePosition = new Transform3d(new Translation3d(), new Rotation3d(0, pitch.in(Radians), yaw.in(Radians)))
      .plus(new Transform3d(new Translation3d(distance.in(Meters), 0, 0), new Rotation3d()));
    Transform3d robotRelativePosition = cameraConstants.RobotToCameraTransform.plus(cameraRelativePosition);
    Pose3d robotPose = swerveDrive.getPosition3d();
    Pose3d fieldRelativePosition = robotPose.plus(robotRelativePosition);

    return fieldRelativePosition.getTranslation().toTranslation2d();
  }

  // 0.0 is the NaN/Null value here
  private double getTargetHeight(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners.size() < 4) return 0.0; // Ensure there are enough corners

    corners.sort((a, b) -> Double.compare(a.y, b.y));
    double avgMinY = (corners.get(0).y + corners.get(1).y) / 2.0;
    double avgMaxY = (corners.get(corners.size() - 1).y + corners.get(corners.size() - 2).y) / 2.0;

    double height = avgMaxY - avgMinY;
    return Double.isNaN(height) ? 0.0 : height; // Ensure NaN is handled
  }

  // 0.0 is the NaN/Null value here
  private double getTargetWidth(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners.size() < 4) return 0.0; // Ensure there are enough corners

    corners.sort((a, b) -> Double.compare(a.x, b.x));
    double avgMinX = (corners.get(0).x + corners.get(1).x) / 2.0;
    double avgMaxX = (corners.get(corners.size() - 1).x + corners.get(corners.size() - 2).x) / 2.0;

    double width = avgMaxX - avgMinX;
    return Double.isNaN(width) ? 0.0 : width; // Ensure NaN is handled
  }

  private boolean isValidSphericalObject(
      double pixelHeight, double pixelWidth, double pixelRatio) {
    if (pixelHeight <= 0 || pixelWidth <= 0) return false;
    return Math.abs(pixelRatio - 1.0) <= cameraConstants.SphereTolerance;
  }

  private Distance calculateDistance(double targetFOVRatio) {
    Distance absoluteDistance =
        Meters.of((cameraConstants.SphereDiameter.in(Meters) / 2) / Math.tan(targetFOVRatio));
    return Meters.of(
        Math.sqrt(
            Math.pow(absoluteDistance.in(Meters), 2)
                - Math.pow(
                    Meters.of(cameraConstants.RobotToCameraTransform.getY()).in(Meters)
                        - (cameraConstants.SphereDiameter.in(Meters)) / 2,
                    2)));
  }
}
