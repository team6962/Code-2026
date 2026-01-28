package com.team6962.lib.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

/**
 * Represents a camera used to detect AprilTags for robot pose estimation.
 */
public class AprilTagCamera {
    /**
     * The PhotonCamera representing the physical camera device.
     */
    private final PhotonCamera device;

    /**
     * The PhotonPoseEstimator used to estimate the robot's pose based on
     * AprilTag detections.
     */
    private final PhotonPoseEstimator poseEstimator;

    /**
     * Configuration constants for AprilTag vision processing.
     */
    private final AprilTagVisionConstants visionConstants;

    /**
     * The transform from the robot to the camera.
     */
    private Transform3d transform;

    /**
     * Simulation object for the camera (if in simulation).
     */
    private PhotonCameraSim cameraSim;

    /**
     * VisionSystemSim for adding the camera to simulation (if in simulation).
     */
    private VisionSystemSim visionSystemSim;

    /**
     * Constructs an AprilTagCamera with the specified parameters. The
     * constructed object can then be used to obtain robot pose estimates based
     * on AprilTag detections via {@link #getRobotPoseEstimates()}.
     * 
     * @param cameraConstants Configuration constants for this camera
     * @param visionConstants Configuration constants for AprilTag vision
     * processing
     * @param visionSystemSim VisionSystemSim for adding the camera to
     * simulation (or null if not in simulation)
     */
    public AprilTagCamera(AprilTagCameraConstants cameraConstants, AprilTagVisionConstants visionConstants, VisionSystemSim visionSystemSim) {
        this.visionConstants = visionConstants;
        this.transform = cameraConstants.Transform;

        device = new PhotonCamera(cameraConstants.Name);
        poseEstimator = new PhotonPoseEstimator(visionConstants.FieldLayout, getTransform());

        if (visionSystemSim != null) {
            cameraSim = new PhotonCameraSim(device, visionConstants.CameraSimProperties);
            cameraSim.enableDrawWireframe(visionConstants.DrawWireframes);
            visionSystemSim.addCamera(cameraSim, cameraConstants.Transform);
            
            this.visionSystemSim = visionSystemSim;
        }
    }

    /**
     * Get the {@link PhotonCamera} device.
     * 
     * @return The {@link PhotonCamera} device
     */
    public PhotonCamera getDevice() {
        return device;
    }

    /**
     * Get the name of this camera.
     * 
     * @return The name of this camera
     */
    public String getName() {
        return device.getName();
    }

    /**
     * Get the robot pose estimator for this camera.
     * 
     * @return The pose estimator
     */
    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    /**
     * Get the transform from the robot to the camera.
     * 
     * @return The transform from the robot to the camera
     */
    public Transform3d getTransform() {
        return transform;
    }

    /**
     * Sets the transform from the robot to the camera.
     * 
     * @param robotToCameraTransform The transform from the robot to the camera
     */
    public void setTransform(Transform3d robotToCameraTransform) {
        transform = robotToCameraTransform;
        
        poseEstimator.setRobotToCameraTransform(transform);

        if (visionSystemSim != null) {
            visionSystemSim.adjustCamera(cameraSim, robotToCameraTransform);
        }
    }

    /**
     * Gets the latest robot pose estimates from the camera's pose estimator.
     * Note that this should be called exactly once per control loop cycle to
     * ensure that all new results are processed and no old results are used.
     * 
     * @return The latest robot pose estimates
     */
    public List<AprilTagVisionMeasurement> getRobotPoseEstimates() {
        List<AprilTagVisionMeasurement> estimates = new ArrayList<>();
        List<PhotonPipelineResult> results = device.getAllUnreadResults();

        for (int i = results.size() - 1; i >= 0; i--) {
            PhotonPipelineResult result = results.get(i);

            Optional<EstimatedRobotPose> possibleEstimate = poseEstimator.estimateCoprocMultiTagPose(result);

            if (possibleEstimate.isEmpty()) {
                possibleEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            if (possibleEstimate.isEmpty()) {
                continue;
            }

            EstimatedRobotPose estimate = possibleEstimate.get();

            Optional<Matrix<N4, N1>> possibleStdDevs = getStandardDeviations(estimate, result.getTargets());

            if (possibleStdDevs.isEmpty()) {
                continue;
            }

            Matrix<N4, N1> stdDevs = possibleStdDevs.get();

            estimates.add(new AprilTagVisionMeasurement(estimate, stdDevs));
        }

        return estimates;
    }

    /**
     * Calculates new standard deviations. This algorithm is a heuristic that
     * creates dynamic standard deviations based on number of tags and distance
     * from the tags.
     * 
     * <p>This code was adapted from the <a href="https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java#L125-L165"><code>updateEstimationStdDevs</code></a>
     * method in PhotonLib's example code.
     * 
     * @param estimatedPose The estimated pose to guess standard deviations for
     * @param targets All targets in this camera frame
     * @return The calculated standard deviations
     */
    private Optional<Matrix<N4, N1>> getStandardDeviations(EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
        int tagCount = 0;
        double totalTagDistance = 0;

        // See how many tags we found, and calculate average distance from the
        // tags to the estimated robot pose
        for (PhotonTrackedTarget target : targets) {
            Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());

            if (tagPose.isEmpty()) continue;

            tagCount++;
            totalTagDistance += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimatedPose.estimatedPose.getTranslation().toTranslation2d());
        }

        // If no tags found, return empty
        if (tagCount == 0) {
            return Optional.empty();
        }

        // Single tag visible and far away - don't trust it
        if (tagCount == 1 && totalTagDistance > visionConstants.MaxSingleTagDistance) {
            return Optional.empty();
        }

        // Divide total distance by number of tags to get average
        double averageTagDistance = totalTagDistance / tagCount;

        // Select base std devs based on number of tags. You don't really
        // need to consider cases where >2 tags are visible seperately, since
        // that is rare.
        Matrix<N4, N1> estStdDevs = tagCount > 1 ? visionConstants.MultiTagStdDevs : visionConstants.SingleTagStdDevs;

        // Increase std devs based on average distance
        estStdDevs = estStdDevs.times(1 + (averageTagDistance * averageTagDistance) * visionConstants.StdDevDistanceScalar);

        return Optional.of(estStdDevs);
    }
}
