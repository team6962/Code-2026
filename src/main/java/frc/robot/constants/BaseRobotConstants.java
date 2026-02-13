package frc.robot.constants;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import com.team6962.lib.vision.AprilTagVisionConstants;
import com.team6962.lib.vision.SphereCameraConstants;

public class BaseRobotConstants implements RobotConstants {
  @Override
  public DrivetrainConstants getDrivetrainConstants() {
    return new DrivetrainConstants();
  }

  @Override
  public AprilTagVisionConstants getAprilTagVisionConstants() {
    return new AprilTagVisionConstants();
  }

  @Override
  public SphereCameraConstants getSphereCameraConstants() {
    return new SphereCameraConstants();
  }

  @Override
  public XBoxTeleopSwerveConstants getTeleopSwerveConstants() {
    return new XBoxTeleopSwerveConstants();
  }

  @Override
  public EnabledSystems getEnabledSystems() {
    return new EnabledSystems();
  }
}
