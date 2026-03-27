package frc.robot.controls;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Utility class for logging controller-related information, such as mapping controller ports to
 * human-readable names for logging purposes.
 */
public class ControllerLogging {
  /**
   * Logs the inputs of the given XboxController to DogLog. This includes the state of all buttons,
   * axes, triggers, and the POV. The inputs are logged under a path that includes the controller's
   * human-readable name based on its port number, which is determined by the {@link
   * #getControllerName(int)} method.
   *
   * @param controller The XboxController whose inputs are to be logged.
   */
  public static void logInputs(XboxController controller) {
    String basePath = "Controllers/" + getControllerName(controller.getPort()) + "/";
    DogLog.log(basePath + "LeftX", controller.getLeftX());
    DogLog.log(basePath + "LeftY", controller.getLeftY());
    DogLog.log(basePath + "RightX", controller.getRightX());
    DogLog.log(basePath + "RightY", controller.getRightY());
    DogLog.log(basePath + "LeftTrigger", controller.getLeftTriggerAxis());
    DogLog.log(basePath + "RightTrigger", controller.getRightTriggerAxis());
    DogLog.log(basePath + "A", controller.getAButton());
    DogLog.log(basePath + "B", controller.getBButton());
    DogLog.log(basePath + "X", controller.getXButton());
    DogLog.log(basePath + "Y", controller.getYButton());
    DogLog.log(basePath + "LeftBumper", controller.getLeftBumperButton());
    DogLog.log(basePath + "RightBumper", controller.getRightBumperButton());
    DogLog.log(basePath + "Back", controller.getBackButton());
    DogLog.log(basePath + "Start", controller.getStartButton());
    DogLog.log(basePath + "LeftStick", controller.getLeftStickButton());
    DogLog.log(basePath + "RightStick", controller.getRightStickButton());
    DogLog.log(basePath + "Connected", controller.isConnected());
    DogLog.log(basePath + "POV", controller.getPOV());
  }

  /**
   * Returns a human-readable name for a controller based on its port number. This is used for
   * logging purposes to make it easier to identify which controller is being referred to in log
   * messages. By convention, port 0 is labeled "Driver" and port 1 is labeled "Operator". Any other
   * ports are labeled as "Controller" followed by the port number.
   *
   * @param port The port number of the controller.
   * @return A human-readable name for the controller.
   */
  public static String getControllerName(int port) {
    if (port == 0) return "Driver";
    else if (port == 1) return "Operator";
    else return "Controller" + port;
  }
}
