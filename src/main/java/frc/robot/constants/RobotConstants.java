package frc.robot.constants;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import com.team6962.lib.vision.AprilTagVisionConstants;
import com.team6962.lib.vision.SphereCameraConstants;

public interface RobotConstants {
  public DrivetrainConstants getDrivetrainConstants();

  public XBoxTeleopSwerveConstants getTeleopSwerveConstants();

  public AprilTagVisionConstants getAprilTagVisionConstants();

  public SphereCameraConstants getSphereCameraConstants();

  public EnabledSystems getEnabledSystems();

  public default RobotConstants withPreferences() {
    return new PreferencedRobotConstants(this);
  }
}
