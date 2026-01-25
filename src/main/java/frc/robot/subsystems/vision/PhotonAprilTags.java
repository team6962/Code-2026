package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.swerve.auto.PoseEstimator;
//import com.team6962.lib.telemetry.logger;
import java.util.logging.Logger;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.Constants.PHOTONVISION;
//import frc.robot.Constants.Field;
import frc.robot.learnbot.LearnBotConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class photonAprilTags extends SubsystemBase {
  private static final double MAX_ROTATION_ERROR = Units.degreesToRadians(15);
  private static final double LARGE_ROTATION_ERROR = 9999999;
  private static final double TRANSLATION_ERROR_FACTOR = 10;
  private static final double ADDITIONAL_TRANSLATION_ERROR = 0.5;

  private static record BestEstimate(
      Pose2d pose, Time timestamp, Distance translationError, Angle rotationError, int tagCount) {
    public BestEstimate() {
      this(
          new Pose2d(),
          Seconds.of(Timer.getFPGATimestamp()),
          Meters.of(Double.POSITIVE_INFINITY),
          Rotations.of(Double.POSITIVE_INFINITY),
          0);
    }
  }

  public static void injectVisionData(
      Map<String, Pose3d> cameraPoses, PoseEstimator poseEstimator) {
    List<Optional<EstimatedRobotPose>> poseEstimates = getPoseEstimates(cameraPoses);

    BestEstimate bestPoseEstimate = new BestEstimate();
    List<Pose2d> loggedVisionPoses = new ArrayList<>();

    for (Optional<EstimatedRobotPose> poseEstimate : poseEstimates) {
      if (poseEstimate.isEmpty()) continue;
      Pose3d pose3d = poseEstimate.get().estimatedPose;
      Pose2d pose2d = pose3d.toPose2d();
      if (isInvalidPoseEstimate(poseEstimate.get(), pose2d)) continue;

      boolean canChangeHeading = canChangeHeading(poseEstimate.get(), poseEstimator, pose2d);

      double rotationError = canChangeHeading ? MAX_ROTATION_ERROR : LARGE_ROTATION_ERROR;
      if (!canChangeHeading) {
        pose2d = adjustPoseRotation(poseEstimator, poseEstimate.get(), pose2d);
      }

      double translationError = calculateTranslationError(poseEstimate.get());

      loggedVisionPoses.add(pose2d);
      translationError += ADDITIONAL_TRANSLATION_ERROR;

      if (isBetterEstimate(bestPoseEstimate, translationError, rotationError)) {
        bestPoseEstimate =
            new BestEstimate(
                pose2d,
                Seconds.of(poseEstimate.get().timestampSeconds),
                Meters.of(translationError),
                Rotations.of(rotationError),
                poseEstimate.get().targetsUsed.size());
      }
    }

    if (bestPoseEstimate.tagCount > 0) {
      poseEstimator.addVisionMeasurement(
          bestPoseEstimate.pose,
          bestPoseEstimate.timestamp,
          VecBuilder.fill(
              bestPoseEstimate.translationError.in(Meters),
              bestPoseEstimate.translationError.in(Meters),
              bestPoseEstimate.rotationError.in(Radians)));
    }

    Logger.getField().getObject("visionPoses").setPoses(loggedVisionPoses);
  }

  private static List<Optional<EstimatedRobotPose>> getPoseEstimates(
      Map<String, Pose3d> cameraPoses) {

    return cameraPoses.entrySet().stream()
        .map(
            entry -> {
              String cameraName = entry.getKey();
              Pose3d robotToCamPose3d = entry.getValue();
              Transform3d robotToCamTransform3d =
                  new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), robotToCamPose3d);
              try (PhotonCamera camera = new PhotonCamera(cameraName)) {
                PhotonPoseEstimator photonPoseEstimator =
                    new PhotonPoseEstimator(
                        Field.FIELD_LAYOUT,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        robotToCamTransform3d);
                Optional<EstimatedRobotPose> estimatedRobotPose =
                    photonPoseEstimator.update(camera.getLatestResult());
                return estimatedRobotPose;
              }
            })
        .collect(Collectors.toList());
  }

  private static boolean isInvalidPoseEstimate(EstimatedRobotPose poseEstimate, Pose2d pose2d) {
    boolean hasBlacklistedTag =
        poseEstimate.targetsUsed.stream()
            .anyMatch(
                target ->
                    IntStream.of(PHOTONVISION.BLACKLISTED_APRILTAGS)
                        .anyMatch(x -> x == target.getFiducialId()));
    double avgTagDist = photonAprilTags.avgTagDistance(poseEstimate);
    return hasBlacklistedTag
        || poseEstimate.targetsUsed.size() == 0
        || pose2d.getTranslation().getNorm() == 0.0
        || pose2d.getRotation().getRadians() == 0.0
        || avgTagDist == 0.0
        || pose2d.getX() < 0.0
        || pose2d.getY() < 0.0
        || pose2d.getX() > Field.LENGTH
        || pose2d.getY() > Field.WIDTH;
  }

  private static boolean canChangeHeading(
      EstimatedRobotPose poseEstimate, PoseEstimator poseEstimator, Pose2d pose2d) {
    boolean canChangeHeading = poseEstimate.targetsUsed.size() >= 2 || RobotState.isDisabled();
    return canChangeHeading
        && poseEstimator.getEstimatedPose().getTranslation().getDistance(pose2d.getTranslation())
            < 1.0;
  }

  private static Pose2d adjustPoseRotation(
      PoseEstimator poseEstimator, EstimatedRobotPose poseEstimate, Pose2d pose2d) {
    return new Pose2d(
        pose2d.getTranslation(),
        poseEstimator.getEstimatedPose(Seconds.of(poseEstimate.timestampSeconds)).getRotation());
  }

  private static double calculateTranslationError(EstimatedRobotPose poseEstimate) {
    return Math.pow(Math.abs(photonAprilTags.avgTagDistance(poseEstimate)), 2.0)
        / Math.pow(poseEstimate.targetsUsed.size(), 2)
        / TRANSLATION_ERROR_FACTOR;
  }

  private static boolean isBetterEstimate(
      BestEstimate bestPoseEstimate, double translationError, double rotationError) {
    return translationError < bestPoseEstimate.translationError.in(Meters)
        || rotationError < Units.degreesToRadians(360.0);
  }

  private static double avgTagDistance(EstimatedRobotPose poseEstimate) {
    return poseEstimate.targetsUsed.stream()
        .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
        .average()
        .orElse(0.0);
  }

  public static void printConfig(Map<String, Pose3d> cameraPoses) {
    System.out.println(
        """


//////////////////////////////////////////
////// PHOTONVISION POSITION CONFIG //////
//////////////////////////////////////////
""");
    for (Map.Entry<String, Pose3d> camera : cameraPoses.entrySet()) {
      System.out.println(
          String.format(
              """
          ----- %s.local:5801 -----
            LL Forward: %.5f
            LL Right:   %.5f
            LL Up:      %.5f
            LL Roll:    %.5f
            LL Pitch:   %.5f
            LL Yaw:     %.5f
          """,
              camera.getKey(),
              camera.getValue().getTranslation().getX(),
              camera.getValue().getTranslation().getY(),
              camera.getValue().getTranslation().getZ(),
              Units.radiansToDegrees(camera.getValue().getRotation().getX()),
              Units.radiansToDegrees(camera.getValue().getRotation().getY()),
              Units.radiansToDegrees(camera.getValue().getRotation().getZ())));
    }
  }


}
