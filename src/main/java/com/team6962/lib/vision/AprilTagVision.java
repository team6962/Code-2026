package com.team6962.lib.vision;

import com.team6962.lib.swerve.CommandSwerveDrive;
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Subsystem that manages AprilTag-based vision systems for robot localization. This class
 * coordinates multiple cameras to provide pose estimates that are integrated with the swerve
 * drive's odometry.
 */
public class AprilTagVision extends SubsystemBase {
  /** Notifier for periodic vision updates. */
  private final Notifier notifier = new Notifier(this::update);

  /** The swerve drive subsystem to provide vision measurements to. */
  private final CommandSwerveDrive swerveDrive;

  /** The constants that configure the vision system. */
  private final AprilTagVisionConstants visionConstants;

  /** Map of camera names to AprilTagCamera instances. */
  private final Map<String, AprilTagCamera> cameras;

  /** Vision simulation system for testing in simulation mode. */
  private VisionSystemSim visionSystemSim;

  /** The AprilTag field layout used for pose estimation. Loaded from WPILib's built-in layouts. */
  private static AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /**
   * Constructs an AprilTagVision subsystem with the given configuration.
   *
   * @param swerveDrive The swerve drive subsystem to integrate vision measurements with.
   * @param visionConstants Configuration constants for the vision system, including field layout
   *     and camera specifications.
   */
  public AprilTagVision(CommandSwerveDrive swerveDrive, AprilTagVisionConstants visionConstants) {
    this.swerveDrive = swerveDrive;
    this.visionConstants = visionConstants;
    this.cameras = new HashMap<>();

    // Initialize vision simulation if running in simulation mode
    if (RobotBase.isSimulation()) {
      visionSystemSim = new VisionSystemSim("main");
      visionSystemSim.addAprilTags(visionConstants.FieldLayout);
    }

    // Create and register all configured cameras
    for (AprilTagCameraConstants cameraConstants : visionConstants.Cameras) {
      addCamera(new AprilTagCamera(cameraConstants, visionConstants, visionSystemSim));
    }

    DogLog.log("Vision/Cameras/Count", visionConstants.Cameras.size());

    notifier.startPeriodic(0.02);
  }

  /**
   * Adds a camera to the vision system.
   *
   * @param camera The AprilTagCamera to add to the system.
   */
  private void addCamera(AprilTagCamera camera) {
    cameras.put(camera.getName(), camera);
  }

  private void update() {
    // Update vision simulation with current robot pose
    if (visionSystemSim != null) {
      visionSystemSim.update(swerveDrive.getSimulation().getRobotPosition());
    }

    // Determine the origin pose based on alliance color (red and blue have mirrored field layouts)
    Pose3d originPose =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? Pose3d.kZero
            : new Pose3d(
                new Translation3d(fieldLayout.getFieldLength(), fieldLayout.getFieldWidth(), 0),
                new Rotation3d(0, 0, Math.PI));

    DogLog.log("/Vision/origin", originPose);
    int measurements = 0;

    // Collect and integrate vision measurements from all cameras
    for (AprilTagCamera camera : cameras.values()) {
      for (AprilTagVisionMeasurement measurement : camera.getRobotPoseEstimates()) {

        // Adjust the measurement to be relative to the field origin based on alliance color
        measurement = measurement.relativeTo(originPose);

        // Don't update rotation unless the conditions specified in vision constants are met
        boolean canUpdateRotation =
            measurement.getPhotonEstimate().targetsUsed.size()
                >= (RobotState.isEnabled()
                    ? visionConstants.MinTagsForHeadingUpdateWhileEnabled
                    : visionConstants.MinTagsForHeadingUpdateWhileDisabled);

        if (!canUpdateRotation) {
          measurement = measurement.withIgnoredRotation();
        }

        // Add the measurement to the swerve drive's localization system
        swerveDrive.getLocalization().addVisionMeasurement(measurement);
        measurements++;
      }
    }

    DogLog.log("Vision/Measurements", measurements);
  }
}
