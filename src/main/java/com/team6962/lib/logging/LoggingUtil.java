package com.team6962.lib.logging;

import com.ctre.phoenix6.controls.ControlRequest;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.InputStream;
import java.util.Map;
import java.util.Optional;
import java.util.Properties;

/** Utility methods for logging telemetry. */
public class LoggingUtil {
  /**
   * Logs all fields from the given ControlRequest under the specified path.
   *
   * @param path The logging path under which to log the ControlRequest fields.
   * @param controlRequest The ControlRequest whose fields are to be logged.
   */
  public static void log(String path, ControlRequest controlRequest) {
    path = ensureEndsWithSlash(path);

    if (controlRequest == null) {
      return;
    }

    Map<String, String> controlInfo = controlRequest.getControlInfo();

    for (String field : controlRequest.getControlInfo().keySet()) {
      String value = controlInfo.get(field);
      Optional<Double> doubleValue = getDouble(value);

      if (doubleValue.isPresent()) {
        DogLog.log(path + field, doubleValue.get());
      } else {
        DogLog.log(path + field, value);
      }
    }
  }

  /**
   * Converts the given string to an Optional Double.
   *
   * @param str The string to convert.
   * @return An Optional containing the Double value if conversion is successful, or an empty
   *     Optional if conversion fails.
   */
  public static Optional<Double> getDouble(String str) {
    try {
      return Optional.of(Double.parseDouble(str));
    } catch (NumberFormatException e) {
      return Optional.empty();
    }
  }

  /**
   * Returns the given logging path, ensuring that the returned path ends with a slash.
   *
   * @param path The logging path to ensure ends with a slash.
   * @return The logging path, guaranteed to end with a slash.
   */
  public static String ensureEndsWithSlash(String path) {
    if (!path.endsWith("/")) {
      path += "/";
    }

    return path;
  }

  /** Logs git properties such as commit hash and build time. */
  public static void logGitProperties() {
    // Load git properties from classpath
    Properties gitProps = new Properties();

    try (InputStream inputStream =
        LoggingUtil.class.getClassLoader().getResourceAsStream("git.properties")) {
      if (inputStream != null) {
        gitProps.load(inputStream);

        // Log all git properties
        gitProps.forEach(
            (key, value) -> {
              DogLog.log("/Metadata/" + key.toString(), value.toString());

              // Also print to console for debugging
              System.out.println(key + ": " + value);
            });
      } else {
        System.err.println("git.properties not found in classpath");
      }
    } catch (Exception e) {
      System.err.println("Failed to load git properties: " + e.getMessage());
    }
  }

  /**
   * Wraps the given command with logging that indicates when the command starts and ends. Logs are
   * written under the path "Commands/{key}" with a boolean value indicating whether the command is
   * currently running (true) or not (false).
   *
   * @param key A unique key to identify the command in the logs. This should be a descriptive name
   *     for the command being logged (e.g., "Shoot", "DriveToClump", etc.).
   * @param command The Command to wrap with logging.
   * @return A new Command that behaves the same as the input command but also logs its execution
   *     status under "Commands/{key}".
   */
  public static Command logCommand(String key, Command command) {
    DogLog.log("Commands/" + key, false);

    CommandScheduler.getInstance().registerComposedCommands(command);

    return new Command() {
      @Override
      public void initialize() {
        DogLog.log("Commands/" + key, true);
        command.initialize();
      }

      @Override
      public void execute() {
        command.execute();
      }

      @Override
      public boolean isFinished() {
        return command.isFinished();
      }

      @Override
      public void end(boolean interrupted) {
        command.end(interrupted);
        DogLog.log("Commands/" + key, false);
      }
    };
  }
}
