package com.team6962.lib.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * Subsystem responsible for locating a clump (collection) of spherical objects detected by a
 * PhotonVision camera. Converts per-target detections into field-relative 2D positions and picks a
 * best estimate for the clump center by selecting the detection whose sum of distances to the other
 * detections is minimal (heuristic for central detection).
 */
public class SphereClumpLocalization extends SubsystemBase {
  /** Drive object used to get robot pose and to publish visualization markers */
  private MotionSwerveDrive swerveDrive;

  /** PhotonVision camera instance for target detection */
  private PhotonCamera camera;

  /** Constants and transforms specific to the camera and the spherical objects being detected */
  private SphereCameraConstants cameraConstants;

  /** Cached clump position */
  private Translation2d cachedClumpPosition;

  /**
   * The list of simulated sphere positions for testing purposes. This allows us to bypass the
   * camera processing and directly set the detected sphere positions in simulation.
   */
  private List<Translation3d> simulatedSpherePositions = new ArrayList<>();

  /**
   * Construct the sphere clump localization subsystem.
   *
   * @param swerveDrive drive subsystem used for robot pose & field visualization
   * @param cameraConstants calibration values (FOV, transform, sphere diameter, class id, etc.)
   */
  public SphereClumpLocalization(
      MotionSwerveDrive swerveDrive, SphereCameraConstants cameraConstants) {
    this.swerveDrive = swerveDrive;
    this.camera =
        !cameraConstants.Name.equals("") && RobotBase.isReal()
            ? new PhotonCamera(cameraConstants.Name)
            : null;
    this.cameraConstants = cameraConstants;
  }

  /**
   * Periodic update called by the scheduler. Attempts to find the clump position and, if found,
   * updates a field object named "Clump" used for visualization on the field logger.
   */
  @Override
  public void periodic() {
    if (cameraConstants.Name.equals("")) return;

    cachedClumpPosition = computeClumpPosition();

    if (cachedClumpPosition != null) {
      // update field visualization with the computed clump pose (zero rotation)
      swerveDrive
          .getFieldLogger()
          .getField()
          .getObject("Clump")
          .setPose(new Pose2d(cachedClumpPosition, new Rotation2d()));
    }
  }

  /**
   * Gets the most recently computed clump position. This is updated periodically by the periodic()
   * method, which queries the camera and processes detections.
   *
   * @return the best-guess clump position as a Translation2d, or null if no clump is detected
   */
  public Translation2d getClumpPosition() {
    return cachedClumpPosition;
  }

  public void setSimulatedSpherePositions(List<Translation3d> spherePositions) {
    if (RobotBase.isSimulation()) {
      simulatedSpherePositions = spherePositions;
    }
  }

  /**
   * Query the camera for targets and compute a best-guess clump position.
   *
   * <p>Algorithm:
   *
   * <ol>
   *   <li>Get latest result from PhotonVision
   *   <li>Convert each valid tracked target into a field-relative 3D translation (then 2D)
   *   <li>Publish all detected spheres to the field visualization
   *   <li>Choose the detection whose sum of pairwise planar distances to other detections (up to
   *       MaxTargets) is smallest
   *   <li>Return that detection's 2D position
   * </ol>
   *
   * @return best estimated clump position as a Translation2d, or null if none found/valid
   */
  private Translation2d computeClumpPosition() {
    PhotonPipelineResult result = null;
    String cameraName = cameraConstants.Name;

    if (RobotBase.isReal()) {
      // TODO: Switch to getAllUnreadResults()
      result = camera.getLatestResult();
      DogLog.log("Vision/Cameras/" + cameraName + "/Spheres/HasTargets", result.hasTargets());
    }

    DogLog.log(
        "Vision/Cameras/" + cameraName + "/Pose",
        swerveDrive.getPosition3d().plus(cameraConstants.RobotToCameraTransform));

    if (RobotBase.isReal() && !result.hasTargets()) {
      DogLog.log("Vision/Cameras/" + cameraName + "/Spheres/Count", 0);
      return null;
    }

    // Get all sphere locations from tracked targets
    List<Translation3d> sphereLocations3d;

    if (RobotBase.isReal()) {
      sphereLocations3d = computeRealSpherePositions(result);
    } else {
      sphereLocations3d = computeSimulatedSpherePositions();
    }

    List<Translation2d> sphereLocations2d =
        sphereLocations3d.stream().map(t -> t.toTranslation2d()).toList();

    // Publish all detected spheres for visualization (Pose2d list)
    logSpheres(sphereLocations2d);

    // Select the detection with the smallest summed planar distance to other detections.
    // This heuristically picks a detection near the clump center.
    return findClump(sphereLocations2d);
  }

  private List<Translation3d> computeRealSpherePositions(PhotonPipelineResult result) {
    List<Translation3d> spherePositions = new ArrayList<>();

    List<PhotonTrackedTarget> targets = result.getTargets();

    int index = 0;

    for (PhotonTrackedTarget target : targets) {
      if (target == null) continue;

      Translation3d position = getSpherePosition(target, index);

      DogLog.log(
          "Vision/Cameras/" + cameraConstants.Name + "/Spheres/" + index + "/IsValid",
          position != null);

      if (position != null) {
        spherePositions.add(position);

        DogLog.log(
            "Vision/Cameras/"
                + cameraConstants.Name
                + "/Spheres/"
                + index
                + "/FieldRelativePosition",
            new Pose3d(position, new Rotation3d()));
      }

      index++;
    }

    return spherePositions;
  }

  /**
   * Computes simulated sphere positions, filtering out any that are outside the camera's maximum
   * simulated detection range or outside the camera's field of view based on the robot's current
   * position and orientation. This allows us to test the clump localization logic in simulation by
   * directly setting sphere positions and having the subsystem process them as if they were
   * detected by the camera.
   *
   * @return a list of valid simulated sphere positions
   */
  private List<Translation3d> computeSimulatedSpherePositions() {
    List<Translation3d> spherePositions = new ArrayList<>();

    int index = 0;

    for (Translation3d position : simulatedSpherePositions) {
      if (swerveDrive.getPosition2d().getTranslation().getDistance(position.toTranslation2d())
          > Math.min(
              cameraConstants.MaxSimulatedDetectionRange.in(Meters),
              cameraConstants.MaxDetectionRange.in(Meters))) {
        continue; // Skip simulated spheres that are out of range
      }

      Pose3d robotPose = swerveDrive.getPosition3d();
      Pose3d cameraPose = robotPose.plus(cameraConstants.RobotToCameraTransform);
      Pose3d sphereRelativeToCamera = new Pose3d(position, new Rotation3d()).relativeTo(cameraPose);

      Angle yaw =
          Radians.of(Math.atan2(sphereRelativeToCamera.getY(), sphereRelativeToCamera.getX()));
      Angle pitch =
          Radians.of(Math.atan2(sphereRelativeToCamera.getZ(), sphereRelativeToCamera.getX()));

      if (Math.abs(yaw.in(Degrees)) > cameraConstants.FOVWidth.getDegrees() / 2
          || Math.abs(pitch.in(Degrees)) > cameraConstants.FOVHeight.getDegrees() / 2) {
        continue; // Skip simulated spheres that are outside the camera's FOV
      }

      spherePositions.add(position);

      DogLog.log(
          "Vision/Cameras/" + cameraConstants.Name + "/Spheres/" + index + "/FieldRelativePosition",
          new Pose3d(position, new Rotation3d()));

      index++;
    }

    return spherePositions;
  }

  /**
   * Helper method to publish detected sphere positions to the field logger for visualization.
   * Converts a list of Translation2d positions into Pose2d with zero rotation and updates a field
   * object named "Sphere". Also logs the count of detected spheres.
   *
   * @param spheres a list of 2D positions of detected spheres
   */
  private void logSpheres(List<Translation2d> spheres) {
    swerveDrive
        .getFieldLogger()
        .getField()
        .getObject("Sphere")
        .setPoses(spheres.stream().map(t -> new Pose2d(t, new Rotation2d())).toList());

    DogLog.log("Vision/Cameras/" + cameraConstants.Name + "/Spheres/Count", spheres.size());
  }

  /**
   * Given a list of detected sphere positions, find the one that is most likely to be at the center
   * of a clump by selecting the position with the lowest total distance to the other positions.
   * This is a heuristic that assumes the clump center will be the detection closest to the other
   * detections.
   *
   * @param sphereLocations2d a list of 2D positions of detected spheres
   * @return the 2D position of the sphere most likely at the center of the clump, or null if
   *     spherePositions was empty
   */
  private Translation2d findClump(List<Translation2d> sphereLocations2d) {
    Translation2d bestPosition = null;
    double lowestTotalDistance = Double.MAX_VALUE;

    for (int i = 0; i < Math.min(sphereLocations2d.size(), cameraConstants.MaxTargets); i++) {
      double totalDistance = 0;
      Translation2d thisSpherePosition = sphereLocations2d.get(i);

      for (int j = 0; j < Math.min(sphereLocations2d.size(), cameraConstants.MaxTargets); j++) {
        if (i == j) continue;

        Translation2d otherSpherePosition = sphereLocations2d.get(j);

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
   * Convert a PhotonTrackedTarget (single detection) into a field-relative 3D translation.
   *
   * <p>Steps:
   *
   * <ol>
   *   <li>Validate detection class id and approximate sphere geometry
   *   <li>Read reported yaw/pitch angles from coprocessor
   *   <li>Estimate distance to target from apparent size (area -> pixel diameter) and the
   *       calibrated focal length
   *   <li>Build a camera-relative transform by placing the sphere along the camera's +X axis at the
   *       computed distance, then rotating the translation by the measured pitch and yaw
   *   <li>Transform from camera-relative to robot-relative using RobotToCameraTransform
   *   <li>Transform from robot-relative to field-relative using the robot's current 3D pose
   * </ol>
   *
   * @param target the tracked target from PhotonVision
   * @param loggingIndex index used for logging. If set to -1, no logging will occur for this
   *     target. If multiple targets are processed, a unique index should be provided for each to
   *     distinguish them in the logs.
   * @return 3D field-relative translation, or null if invalid/out of range
   */
  private Translation3d getSpherePosition(PhotonTrackedTarget target, int loggingIndex) {
    if (target == null || target.getDetectedObjectClassID() != cameraConstants.ClassId) {
      return null; // Check if detection is valid
    }

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
        cameraConstants.FOVHeight.getRadians() * (pixelHeight / cameraConstants.CameraHeightPixels);
    // If angular size is larger than 90 degrees, discard as invalid
    if (angularHeight > Math.PI / 2) return null;

    // Estimate distance from the camera to the sphere using apparent size and known diameter
    Distance distance = calculateDistance(target);

    if (loggingIndex != -1) {
      DogLog.log(
          "Vision/Cameras/"
              + cameraConstants.Name
              + "/Spheres/"
              + loggingIndex
              + "/CameraToTargetDistance",
          calculateDistance(target));
    }

    if (distance.gt(cameraConstants.MaxDetectionRange)) return null;

    /*
     * Build a transform that represents the object's pose relative to the camera:
     * - Rotation3d(0, pitch, yaw) sets the orientation so the camera's +X axis points toward the
     *   detected object given measured pitch (rotation about Y) and yaw (rotation about Z).
     * - Translate along the camera's +X axis by the computed distance so the object's center lies
     *   in front of the camera at that range.
     */
    Transform3d cameraRelativePosition =
        new Transform3d(new Translation3d(), new Rotation3d(0, pitch.in(Radians), yaw.in(Radians)))
            .plus(new Transform3d(new Translation3d(distance.in(Meters), 0, 0), new Rotation3d()));

    if (loggingIndex != -1) {
      DogLog.log(
          "Vision/Cameras/"
              + cameraConstants.Name
              + "/Spheres/"
              + loggingIndex
              + "/CameraRelativePosition",
          new Pose3d(
              cameraRelativePosition.getTranslation(), cameraRelativePosition.getRotation()));
    }

    // Transform from camera-relative to robot-relative, then robot to field using current robot
    // pose
    Transform3d robotRelativePosition =
        cameraConstants.RobotToCameraTransform.plus(cameraRelativePosition);

    if (loggingIndex != -1) {
      DogLog.log(
          "Vision/Cameras/"
              + cameraConstants.Name
              + "/Spheres/"
              + loggingIndex
              + "/RobotRelativePosition",
          new Pose3d(robotRelativePosition.getTranslation(), robotRelativePosition.getRotation()));
    }

    Pose3d robotPose = swerveDrive.getPosition3d();
    Pose3d fieldRelativePosition = robotPose.plus(robotRelativePosition);

    return fieldRelativePosition.getTranslation();
  }

  /**
   * Compute a stable pixel height for the target using the min-area-rect corners reported by
   * PhotonVision. Uses averaging of the top two and bottom two corners to reduce jitter.
   *
   * @param target the tracked target from PhotonVision
   * @return pixel height (or 0 if failed to compute valid height)
   */
  private double getTargetHeight(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners.size() < 4) return 0; // Ensure there are enough corners

    // Sort by Y to find top and bottom corners, then average top two and bottom two for robustness
    corners = corners.stream().sorted((a, b) -> Double.compare(a.y, b.y)).toList();
    double avgMinY = (corners.get(0).y + corners.get(1).y) / 2.0;
    double avgMaxY = (corners.get(corners.size() - 1).y + corners.get(corners.size() - 2).y) / 2.0;

    double height = avgMaxY - avgMinY;
    return Double.isNaN(height) ? 0.0 : height; // Ensure NaN is handled
  }

  /**
   * Compute a stable pixel width for the target using the min-area-rect corners reported by
   * PhotonVision. Uses averaging of the left two and right two corners to reduce jitter.
   *
   * @param target the tracked target from PhotonVision
   * @return pixel width (or 0 if failed to compute valid width)
   */
  private double getTargetWidth(PhotonTrackedTarget target) {
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    if (corners.size() < 4) return 0; // Ensure there are enough corners

    // Sort by X to find left and right corners, then average left two and right two for robustness
    corners = corners.stream().sorted((a, b) -> Double.compare(a.x, b.x)).toList();
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
  private boolean isValidSphericalObject(double pixelHeight, double pixelWidth, double pixelRatio) {
    if (pixelHeight <= 0 || pixelWidth <= 0) return false;
    return Math.abs(pixelRatio - 1.0) <= cameraConstants.SphereTolerance;
  }

  /**
   * Estimate the straight-line distance from the camera to the detected sphere center using the
   * detected area and the calibrated focal length values.
   *
   * <p>Implementation:
   *
   * <ol>
   *   <li>Convert reported area (fraction of image area) to an effective pixel diameter estimate
   *       (observedDiameter) assuming a circular projection: observedDiameter = sqrt(areaFraction *
   *       imagePixels)
   *   <li>Compute mean focal length from calibrated focal lengths in X and Y
   *   <li>Use pinhole camera model: distance = (realSphereDiameter * focalLength) /
   *       observedDiameter
   * </ol>
   *
   * @param target PhotonTrackedTarget used to obtain measured area
   * @return straight-line Distance from camera to sphere center (meters)
   */
  private Distance calculateDistance(PhotonTrackedTarget target) {
    double observedDiameter =
        Math.sqrt(
            (target.getArea() / 100)
                * cameraConstants.CameraHeightPixels
                * cameraConstants.CameraWidthPixels);
    double meanFocalLength =
        Math.sqrt(cameraConstants.CameraFocalLengthX * cameraConstants.CameraFocalLengthY);
    Distance distanceMeters =
        (Meters.of((cameraConstants.SphereDiameter).in(Meters)).times(meanFocalLength))
            .div(observedDiameter);
    return distanceMeters;
  }
}
