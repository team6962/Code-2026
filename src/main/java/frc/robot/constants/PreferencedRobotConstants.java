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

  public PreferencedRobotConstants(RobotConstants internalConstants) {
    drivetrainConstants = internalConstants.getDrivetrainConstants();
    teleopSwerveConstants = internalConstants.getTeleopSwerveConstants();
    aprilTagVisionConstants = internalConstants.getAprilTagVisionConstants();
    sphereCameraConstants = internalConstants.getSphereCameraConstants();

    drivetrainConstants.Simulation.EnablePoseEstimation =
        Preferences.enablePoseEstimationInSimulation;
    teleopSwerveConstants.ReorientControlsInSimulation = Preferences.reorientControlsInSimulation;
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
}
