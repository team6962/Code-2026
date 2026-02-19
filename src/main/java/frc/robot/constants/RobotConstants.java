package frc.robot.constants;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import com.team6962.lib.vision.AprilTagVisionConstants;
import com.team6962.lib.vision.SphereCameraConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Preferences;

public interface RobotConstants {
  public DrivetrainConstants getDrivetrainConstants();

  public XBoxTeleopSwerveConstants getTeleopSwerveConstants();

  public AprilTagVisionConstants getAprilTagVisionConstants();

  public SphereCameraConstants getSphereCameraConstants();

  public EnabledSystems getEnabledSystems();

  public static RobotConstants generate() {
    RobotConstants constants;

    if (RobotBase.isSimulation()) {
      constants = Preferences.simulatedRobot;
    } else {
      String robotName = edu.wpi.first.wpilibj.Preferences.getString("Robot", "[unset]");

      if (robotName.toLowerCase().equals("learnbot")) {
        constants = new LearnBotConstants();
      } else if (robotName.toLowerCase().equals("competition")) {
        constants = new CompetitionBotConstants();
      } else {
        DriverStation.reportError(
            "Unknown robot specified in preferences: "
                + robotName
                + ". Valid options are: LearnBot, Competition. Using competition robot constants by default",
            false);

        constants = new CompetitionBotConstants();
      }
    }

    constants = new PreferencedRobotConstants(constants);

    return constants;
  }
}
