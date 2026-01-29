package com.team6962.lib.vision;

import com.team6962.lib.swerve.CommandSwerveDrive;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Subsystem that manages AprilTag-based vision systems for robot localization. This class
 * coordinates multiple cameras to provide pose estimates that are integrated with the swerve
 * drive's odometry.
 */
public class AprilTagVision extends SubsystemBase {
  /** The swerve drive subsystem to provide vision measurements to. */
  private final CommandSwerveDrive swerveDrive;

  /** Map of camera names to AprilTagCamera instances. */
  private final Map<String, AprilTagCamera> cameras;

  /** Vision simulation system for testing in simulation mode. */
  private VisionSystemSim visionSystemSim;

  /**
   * Constructs an AprilTagVision subsystem with the given configuration.
   *
   * @param swerveDrive The swerve drive subsystem to integrate vision measurements with.
   * @param visionConstants Configuration constants for the vision system, including field layout
   *     and camera specifications.
   */
  public AprilTagVision(CommandSwerveDrive swerveDrive, AprilTagVisionConstants visionConstants) {
    this.swerveDrive = swerveDrive;
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

    DogLog.log("Vision/CameraCount", visionConstants.Cameras.size());
  }

  /**
   * Adds a camera to the vision system.
   *
   * @param camera The AprilTagCamera to add to the system.
   */
  public void addCamera(AprilTagCamera camera) {
    cameras.put(camera.getName(), camera);
  }

  /**
   * Gets an immutable list of all cameras in the vision system.
   *
   * @return A list containing all registered AprilTagCamera instances.
   */
  public List<AprilTagCamera> getCameras() {
    return List.copyOf(cameras.values());
  }

  /**
   * Retrieves a camera by name.
   *
   * @param name The name of the camera to retrieve.
   * @return The AprilTagCamera with the given name, or null if not found.
   */
  public AprilTagCamera getCamera(String name) {
    return cameras.get(name);
  }

  @Override
  public void periodic() {
    // Update vision simulation with current robot pose
    if (visionSystemSim != null) {
      visionSystemSim.update(swerveDrive.getSimulation().getOdometry().getPosition());
    }

    int measurements = 0;

    // Collect and integrate vision measurements from all cameras
    for (AprilTagCamera camera : cameras.values()) {
      for (AprilTagVisionMeasurement measurement : camera.getRobotPoseEstimates()) {
        swerveDrive.getLocalization().addVisionMeasurement(measurement);
        measurements++;
      }
    }

    DogLog.log("Vision/Measurements", measurements);
  }
}
