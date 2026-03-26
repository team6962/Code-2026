package frc.robot.controls;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;

/** Provides feedback to the driver through controller rumble based on match state and timing. */
public class ShiftFeedback extends SubsystemBase {
  private static double NEARLY_ZERO = 0.001;

  /** The start of teleop. */
  private double teleopStartTimeSeconds = -1;

  /** Whether to simulate the endgame phase when not in practice mode or an official match. */
  private boolean includeEndgameInTesting = false;

  /**
   * Whether to simulate winning the autonomous phase when not in practice mode or an official
   * match.
   */
  private boolean winAutoInTesting = false;

  /**
   * Creates a new ShiftFeedback subsystem and schedules rumble commands on the provided
   * controllers.
   *
   * @param controllers The list of controllers to schedule rumble commands on.
   */
  public ShiftFeedback(List<ControllerRumble> controllers) {
    for (ControllerRumble controller : controllers) {
      CommandScheduler.getInstance().schedule(controller.rumble(this::getRumbleIntensity));
    }

    DogLog.tunable(
        "ShiftFeedback/IncludeEndgameInTesting",
        includeEndgameInTesting,
        value -> includeEndgameInTesting = value);

    DogLog.tunable(
        "ShiftFeedback/WinAutoInTesting", winAutoInTesting, value -> winAutoInTesting = value);
  }

  /**
   * Calculates the rumble intensity based on the current match state and timing.
   *
   * @return The rumble intensity, ranging from 0.0 to 1.0.
   */
  private double getRumbleIntensity() {
    if (RobotState.isDisabled() || !RobotState.isTeleop())
      return 0.0; // Don't rumble if we're disabled or not in teleop

    double timeUntilSwitchSeconds = getTimeUntilSwitch();

    if (timeUntilSwitchSeconds < 0.0)
      return 0.0; // Don't rumble if we don't know when the next switch is

    double rumble = getRumbleIntensity(timeUntilSwitchSeconds);

    DogLog.log("ShiftFeedback/Rumble", rumble);

    return rumble;
  }

  @Override
  public void periodic() {
    if (RobotState.isEnabled() && teleopStartTimeSeconds < 0) {
      teleopStartTimeSeconds = Timer.getFPGATimestamp();
    } else if (RobotState.isDisabled() && teleopStartTimeSeconds >= 0) {
      teleopStartTimeSeconds = -1;
    }

    if (teleopStartTimeSeconds >= 0.0) {
      double timeSinceTeleopStartSeconds = Timer.getFPGATimestamp() - teleopStartTimeSeconds;

      DogLog.forceNt.log(
          "ShiftFeedback/TeleopElapsedTime",
          Math.min(130, Math.max(NEARLY_ZERO, timeSinceTeleopStartSeconds)));
      DogLog.forceNt.log(
          "ShiftFeedback/TeleopRemainingTime",
          Math.max(NEARLY_ZERO, 130.0 - timeSinceTeleopStartSeconds));
      DogLog.forceNt.log(
          "ShiftFeedback/TimeUntilSwitch", Math.max(NEARLY_ZERO, getTimeUntilSwitch()));
      DogLog.forceNt.log("ShiftFeedback/IsActive", isHubActive());
      DogLog.forceNt.log("ShiftFeedback/HubStatus", getHubStatusColor());
    } else {
      DogLog.forceNt.log("ShiftFeedback/TeleopElapsedTime", NEARLY_ZERO);
      DogLog.forceNt.log("ShiftFeedback/TeleopRemainingTime", NEARLY_ZERO);
      DogLog.forceNt.log("ShiftFeedback/TimeUntilSwitch", NEARLY_ZERO);
      DogLog.forceNt.log("ShiftFeedback/IsActive", false);
      DogLog.forceNt.log("ShiftFeedback/HubStatus", "#888888");
    }
  }

  private String getHubStatusColor() {
    return isHubActive() ? "#00ff00" : "#ff0000";
  }

  /**
   * Returns whether the hub is currently active based on the current robot state and match time.
   * When not in a match, the hub will stay active at all times.
   *
   * @return Whether the hub is currently active.
   */
  private boolean isHubActive() {
    if (RobotState.isDisabled()) {
      return false;
    } else if (RobotState.isAutonomous()) {
      return true;
    } else if (RobotState.isTeleop()) {
      double timeSinceTeleopStart = Timer.getFPGATimestamp() - teleopStartTimeSeconds;

      if (timeSinceTeleopStart <= 100.0) {
        int shiftNumber = (int) (timeSinceTeleopStart / 25.0);

        return shiftNumber % 2 == (didWinAuto() ? 1 : 0);
      } else {
        return true;
      }
    } else {
      return false;
    }
  }

  /**
   * Returns the time until the next switch between active and inactive, or -1 if unknown.
   *
   * @return The time until the next switch between active and inactive, or -1 if unknown.
   */
  private double getTimeUntilSwitch() {
    double timeSinceTeleopStartSeconds = Timer.getFPGATimestamp() - teleopStartTimeSeconds;

    if ((isPlayingMatch() || includeEndgameInTesting) && timeSinceTeleopStartSeconds > 130.0) {
      return -1;
    }

    if ((isPlayingMatch() || includeEndgameInTesting)
        && (timeSinceTeleopStartSeconds > 100.0
            || (didWinAuto() && timeSinceTeleopStartSeconds > 75.0))) {
      return 130.0 - timeSinceTeleopStartSeconds;
    }

    return 25.0 - (timeSinceTeleopStartSeconds % 25.0);
  }

  /**
   * Gets which alliance scored more fuel in autonomous, according to the FMS, and therefore has an
   * inactive hub during the first shift in teleop.
   *
   * @return An Optional containing the alliance that won fuel in autonomous, or an empty Optional
   *     if the winning alliance is unknown.
   */
  private Optional<Alliance> getAutoWinningAlliance() {
    return DriverStation.getGameSpecificMessage().contains("R")
        ? Optional.of(Alliance.Red)
        : DriverStation.getGameSpecificMessage().contains("B")
            ? Optional.of(Alliance.Blue)
            : Optional.empty();
  }

  /**
   * Determines whether our alliance won autonomous based on the FMS data. This is used to find the
   * initial state of the hub in teleop.
   *
   * @return Whether our alliance won autonomous.
   */
  private boolean didWinAuto() {
    if (isPlayingMatch()) {
      Optional<Alliance> winningAlliance = getAutoWinningAlliance();
      Optional<Alliance> currentAlliance = DriverStation.getAlliance();

      return winningAlliance.isPresent()
          && currentAlliance.isPresent()
          && winningAlliance.get() == currentAlliance.get();
    } else {
      return winAutoInTesting;
    }
  }

  /**
   * Determines whether we're playing a match (i.e. connected to FMS or in practice mode) or
   * practicing outside of a match (i.e. in teleop without FMS).
   *
   * @return True if we're playing a match, false otherwise.
   */
  private boolean isPlayingMatch() {
    return DriverStation.getMatchType() != MatchType.None;
  }

  /**
   * Calculates the rumble intensity based on the remaining time until the next switch.
   *
   * @param remainingTimeSeconds The remaining time until the next switch, in seconds.
   * @return The rumble intensity, ranging from 0.0 (no rumble) to 1.0 (full rumble).
   */
  private double getRumbleIntensity(double remainingTimeSeconds) {
    if (remainingTimeSeconds <= 1.0) {
      return 1.0;
    } else if (remainingTimeSeconds <= 5.0) {
      return Math.floor(remainingTimeSeconds * 2.0) % 2 == 1 ? 1.0 : 0.0;
    } else if (remainingTimeSeconds >= 9.75 && remainingTimeSeconds <= 10.25) {
      return 0.75;
    } else if (remainingTimeSeconds >= 14.75 && remainingTimeSeconds <= 15.25) {
      return 0.5;
    } else if (remainingTimeSeconds >= 19.75 && remainingTimeSeconds <= 20.25) {
      return 0.25;
    }

    return 0.0;
  }
}
