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

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem responsible for locating a clump (collection) of spherical objects detected by a
 * PhotonVision camera. Converts per-target detections into field-relative 2D positions and picks
 * a best estimate for the clump center by selecting the detector with the lowest total pairwise
 * distance to other detections.
 */
public class SphereClumpLocalization extends SubsystemBase {
  // Drive object used to get robot pose and to publish visualization markers
  private MotionSwerveDrive swerveDrive;
  // PhotonVision camera instance for target detection
  private PhotonCamera camera;
  // Constants and transforms specific to the camera and the spherical objects being detected
  private SphereCameraConstants cameraConstants;

  /**
   * Construct the localization subsystem.
   *
   * @param swerveDrive drive subsystem used for robot pose & field visualization
   * @param cameraConstants calibration values (FOV, transform, sphere diameter, class id, etc.)
   */
  public SphereClumpLocalization(MotionSwerveDrive swerveDrive, SphereCameraConstants cameraConstants) {
    this.swerveDrive = swerveDrive;
    this.camera = new PhotonCamera(cameraConstants.Name);
    this.cameraConstants = cameraConstants;
  }

  /**
   * Periodic update called by the scheduler. Attempts to find the clump position and, if found,
   * updates a field object named "Clump" for visualization.
   */
  @Override
  public void periodic() {
    Translation2d clump = getClumpPosition();

    if (clump != null) {
      // update field visualization with the computed clump pose (zero rotation)
      swerveDrive.getFieldLogger().getField().getObject("Clump").setPose(new Pose2d(clump, new Rotation2d()));
    }
  }

  /**
   * Query the camera for targets and compute a best-guess clump position.
   *
   * The algorithm:
   * - Get latest result from PhotonVision
   * - Convert each valid tracked target into a field-relative 2D translation
   * - Publish all detected spheres to the field visualization
   * - Choose the sphere whose sum of distances to the other spheres (up to MaxTargets) is minimal
   *
   * @return best estimated clump position as a Translation2d, or null if none found/valid
   */
  public Translation2d getClumpPosition() {
    // TODO: Switch to getAllUnreadResults()
    PhotonPipelineResult result = camera.getLatestResult();
    DogLog.log("Vision/Cameras/" + camera.getName() + "/Spheres/hasTargets", result.hasTargets());
    if (!result.hasTargets()) return null;

    Translation2d bestPosition = null;
    double lowestTotalDistance = Double.MAX_VALUE;

    List<PhotonTrackedTarget> targets = result.getTargets();
    List<Translation2d> sphereLocations = new ArrayList<>();
    int sphereIndex = 0;
    for (PhotonTrackedTarget target : targets) {
      if (target == null) continue;
      // Convert tracked target into a 2D field relative position
      Translation2d sphereLocation = getSpherePosition(target);
      if (sphereLocation == null) continue;
      sphereLocations.add(sphereLocation);
      DogLog.log("Vision/sphere-location/" + sphereIndex, sphereLocation);
      sphereIndex++;
    }

    // Publish all detected spheres for visualization (Pose2d list)
    swerveDrive.getFieldLogger().getField().getObject("Sphere").setPoses(sphereLocations.stream().map(t -> new Pose2d(t, new Rotation2d())).toList());
    DogLog.log("Vision/Cameras/" + camera.getName() + "/Spheres/sphereCount", sphereIndex);

    // Select the sphere with the smallest summed distance to the other spheres.
    // This heuristically picks a detection near the center of a clump.
    for (int i = 0; i < Math.min(sphereLocations.size(), cameraConstants.MaxTargets); i++) {
      double totalDistance = 0;
      Translation2d thisSpherePosition = sphereLocations.get(i);
      for (int j = 0; j < Math.min(sphereLocations.size(), cameraConstants.MaxTargets); j++) {
        if (i == j) continue;

        Translation2d otherSpherePosition = sphereLocations.get(j);
        if (otherSpherePosition != null) {
          double sphereDistance = thisSpherePosition.getDistance(otherSpherePosition);
          totalDistance = totalDistance + sphereDistance;
        }
      }
      if (totalDistance < lowestTotalDistance) {
        lowestTotalDistance = totalDistance;
        bestPosition = thisSpherePosition;
      }
    }
    return bestPosition;
  }

  /**
   * Convert a PhotonTrackedTarget (single detection) into a field-relative 2D translation.
   *
   * Steps:
   * - Validate detection class id and geometry (approximate sphere)
   * - Compute angles to target from camera yaw/pitch
   * - Estimate range using apparent angular size vs. known sphere diameter
   * - Build a Transform3d from camera coordinates to the detected sphere
   * - Transform to robot coordinates via RobotToCameraTransform and then to field coordinates
   *
   * @param target the tracked target from PhotonVision
   * @return 2D field-relative translation, or null if invalid/out of range
   */
  public Translation2d getSpherePosition(PhotonTrackedTarget target) {
    if (target == null || target.getDetectedObjectClassID() != cameraConstants.ClassId)
      return null; // Check if detection is valid

    // Target yaw and pitch reported by PhotonVision (degrees -> Angle)
    Angle yaw = Degrees.of(target.getYaw());
    Angle pitch = Degrees.of(target.getPitch());

    // Pixel extents for the detected minimum area rect around the target
    double pixelHeight = getTargetHeight(target);
    double pixelWidth = getTargetWidth(target);
    double pixelRatio = pixelHeight / pixelWidth;

    // Filter out non-spherical detections based on pixel ratio and sanity checks
    if (!isValidSphericalObject(pixelHeight, pixelWidth, pixelRatio)) return null;

    // Convert pixel height to angular height relative to the camera vertical FOV
    double angularHeight =
        cameraConstants.FOVHeight.getRadians()
            * (pixelHeight / cameraConstants.CameraHeightPixels);
    // If angular size is larger than 90 degrees, something is wrong => discard
    if (angularHeight > Math.PI / 2) return null;

    // Estimate distance from the camera to the sphere using angular size and known diameter
    Distance distance = calculateDistance(angularHeight);
    if (distance.gt(cameraConstants.MaxDetectionRange)) return null;

    /*
     * Build a transform that represents the object's pose relative to the camera:
     * - Start with rotation of pitch (x-tilt) and yaw (z-rotation) to point at the object
     * - Then translate along the camera's +X axis by the computed distance
     * The composition order places the object in front of the camera at the computed range and
     * oriented by the reported angles.
     */
    Transform3d cameraRelativePosition = new Transform3d(new Translation3d(), new Rotation3d(0, pitch.in(Radians), yaw.in(Radians)))
      .plus(new Transform3d(new Translation3d(distance.in(Meters), 0, 0), new Rotation3d()));

    // Transform from camera-relative to robot-relative, then robot to field using current robot pose
    Transform3d robotRelativePosition = cameraConstants.RobotToCameraTransform.plus(cameraRelativePosition);
    Pose3d robotPose = swerveDrive.getPosition3d();
    Pose3d fieldRelativePosition = robotPose.plus(robotRelativePosition);

    // Return a 2D translation projected to the field plane (ignore Z)
    return fieldRelativePosition.getTranslation().toTranslation2d();
  }

  // 0.0 is the NaN/Null value here
  /**
   * Compute a stable pixel height for the target using the min-area-rect corners reported by
   * PhotonVision. Uses averaging of the top two and bottom two corners to reduce jitter.
   *
   * @return pixel height (or 0.0 on invalid/NaN)
   */
  private double getTargetHeight(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners.size() < 4) return 0.0; // Ensure there are enough corners

    // Sort by Y to find top and bottom corners, then average top two and bottom two for robustness
    corners.sort((a, b) -> Double.compare(a.y, b.y));
    double avgMinY = (corners.get(0).y + corners.get(1).y) / 2.0;
    double avgMaxY = (corners.get(corners.size() - 1).y + corners.get(corners.size() - 2).y) / 2.0;

    double height = avgMaxY - avgMinY;
    return Double.isNaN(height) ? 0.0 : height; // Ensure NaN is handled
  }

  // 0.0 is the NaN/Null value here
  /**
   * Compute a stable pixel width for the target using the min-area-rect corners reported by
   * PhotonVision. Uses averaging of the left two and right two corners to reduce jitter.
   *
   * @return pixel width (or 0.0 on invalid/NaN)
   */
  private double getTargetWidth(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners.size() < 4) return 0.0; // Ensure there are enough corners

    // Sort by X to find left and right corners, then average left two and right two for robustness
    corners.sort((a, b) -> Double.compare(a.x, b.x));
    double avgMinX = (corners.get(0).x + corners.get(1).x) / 2.0;
    double avgMaxX = (corners.get(corners.size() - 1).x + corners.get(corners.size() - 2).x) / 2.0;

    double width = avgMaxX - avgMinX;
    return Double.isNaN(width) ? 0.0 : width; // Ensure NaN is handled
  }

  /**
   * Quick validation that the detected rectangle looks approximately like a sphere in the image.
   * Reject zero/invalid dimensions and enforce a tolerance around a 1:1 pixel aspect ratio.
   *
   * @param pixelHeight height in pixels
   * @param pixelWidth width in pixels
   * @param pixelRatio height/width ratio
   * @return true if the detection looks like a spherical object
   */
  private boolean isValidSphericalObject(
      double pixelHeight, double pixelWidth, double pixelRatio) {
    if (pixelHeight <= 0 || pixelWidth <= 0) return false;
    return Math.abs(pixelRatio - 1.0) <= cameraConstants.SphereTolerance;
  }

  /**
   * Estimate the distance to the sphere from its apparent angular height in the camera.
   *
   * Calculation notes:
   * - The code computes absoluteDistance using SphereDiameter/2 divided
   *   by tan(angularHeight). This effectively treats angularHeight as the angle between the camera
   *   centerline and the sphere top/bottom (geometry approximation).
   * - Then it corrects for the camera's vertical offset from the sphere's center (robot->camera Y)
   *   and computes the horizontal distance via Pythagoras.
   *
   * @param angularHeight angular height (radians)
   * @return Distance object representing the computed range
   */
  private Distance calculateDistance(double angularHeight) {
    Distance absoluteDistance =
        Meters.of((cameraConstants.SphereDiameter.in(Meters) / 2) / Math.tan(angularHeight));
    return Meters.of(
        Math.sqrt(
            Math.pow(absoluteDistance.in(Meters), 2)
                - Math.pow(
                    Meters.of(cameraConstants.RobotToCameraTransform.getY()).in(Meters)
                        - (cameraConstants.SphereDiameter.in(Meters)) / 2,
                    2)));
  }
}
