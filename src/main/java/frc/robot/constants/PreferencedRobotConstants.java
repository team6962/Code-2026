package frc.robot.constants;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import com.team6962.lib.vision.AprilTagVisionConstants;
import com.team6962.lib.vision.SphereCameraConstants;
import frc.robot.Preferences;

public class PreferencedRobotConstants implements RobotConstants {
  private DrivetrainConstants drivetrainConstants;
  private XBoxTeleopSwerveConstants teleopSwerveConstants;
  private AprilTagVisionConstants aprilTagVisionConstants;
  private SphereCameraConstants sphereCameraConstants;
  private EnabledSystems enabledSystems;

  public PreferencedRobotConstants(RobotConstants internalConstants) {
    drivetrainConstants = internalConstants.getDrivetrainConstants();
    teleopSwerveConstants = internalConstants.getTeleopSwerveConstants();
    aprilTagVisionConstants = internalConstants.getAprilTagVisionConstants();
    sphereCameraConstants = internalConstants.getSphereCameraConstants();
    enabledSystems = internalConstants.getEnabledSystems();

    drivetrainConstants.Simulation.EnablePoseEstimation =
        Preferences.enablePoseEstimationInSimulation;
    teleopSwerveConstants.ReorientControlsInSimulation = Preferences.reorientControlsInSimulation;
    enabledSystems = enabledSystems.and(Preferences.enabledSystems);
  }

  @Override
  public DrivetrainConstants getDrivetrainConstants() {
    return drivetrainConstants;
  }

  @Override
  public XBoxTeleopSwerveConstants getTeleopSwerveConstants() {
    return teleopSwerveConstants;
  }

  @Override
  public AprilTagVisionConstants getAprilTagVisionConstants() {
    return aprilTagVisionConstants;
  }

  @Override
  public SphereCameraConstants getSphereCameraConstants() {
    return sphereCameraConstants;
  }

  @Override
  public EnabledSystems getEnabledSystems() {
    return enabledSystems;
  }
}
