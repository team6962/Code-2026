package frc.robot.controls;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;

/** Provides feedback to the driver through controller rumble based on match state and timing. */
public class ShiftFeedback extends SubsystemBase {
  /** The simulated start time of the match, used in simulation mode. */
  private double simulatedMatchStartTime = -1;

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
  }

  /**
   * Calculates the rumble intensity based on the current match state and timing.
   *
   * @return The rumble intensity, ranging from 0.0 to 1.0.
   */
  private double getRumbleIntensity() {
    if (RobotState.isDisabled() || !RobotState.isTeleop())
      return 0.0; // Don't rumble if we're disabled or not in teleop
    if (isPlayingMatch() && getAutoWinningAlliance().isEmpty())
      return 0.0; // Don't rumble if we don't know who won auto

    double timeUntilSwitchSeconds = getTimeUntilSwitch();

    if (timeUntilSwitchSeconds < 0)
      return 0.0; // Don't rumble if we don't know when the next switch is

    double rumble = getRumbleIntensity(timeUntilSwitchSeconds);

    DogLog.log("ShiftFeedback/Rumble", rumble);

    return rumble;
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      if (RobotState.isEnabled() && simulatedMatchStartTime < 0) {
        simulatedMatchStartTime = Timer.getFPGATimestamp();
      } else if (RobotState.isDisabled() && simulatedMatchStartTime >= 0) {
        simulatedMatchStartTime = -1;
      }
    }

    DogLog.log("ShiftFeedback/MatchTime", getMatchTime());
    DogLog.forceNt.log("ShiftFeedback/TimeUntilSwitch", getTimeUntilSwitch());
    DogLog.forceNt.log("ShiftFeedback/IsActive", isHubActive());
    DogLog.forceNt.log(
        "ShiftFeedback/HubStatus",
        isPlayingMatch()
            ? (isHubActive() ? "#00ff00" : "#ff0000")
            : ((int) (getMatchTime() / 25.0) % 2 == 0 ? "#ffcc00" : "#0048ff"));
  }

  /**
   * Returns whether the hub is currently active based on the current robot state and match time.
   * When not in a match, the hub will stay active at all times.
   *
   * @return Whether the hub is currently active.
   */
  private boolean isHubActive() {
    if (isPlayingMatch()) {
      if (RobotState.isDisabled()) {
        return false;
      } else if (RobotState.isAutonomous()) {
        return true;
      } else if (RobotState.isTeleop()) {
        double timeSinceTeleopStart = (25.0 * 4.0 + 30.0) - getMatchTime();

        if (timeSinceTeleopStart <= 100.0) {
          int shiftNumber = (int) (timeSinceTeleopStart / 25.0);

          return shiftNumber % 2 == (didWinAuto() ? 1 : 0);
        } else {
          return true;
        }
      } else {
        return false;
      }
    } else {
      return RobotState.isEnabled();
    }
  }

  /**
   * Returns the time until the next switch between active and inactive, or -1 if unknown.
   *
   * @return The time until the next switch between active and inactive, or -1 if unknown.
   */
  private double getTimeUntilSwitch() {
    if (isPlayingMatch()) {
      if (RobotState.isDisabled()) {
        return -1;
      } else if (RobotState.isAutonomous()) {
        return getMatchTime();
      } else if (RobotState.isTeleop()) {
        double timeSinceTeleopStart = (25.0 * 4.0 + 30.0) - getMatchTime();

        if (timeSinceTeleopStart <= 100.0) {
          int currentShiftNumber = (int) (timeSinceTeleopStart / 25.0);
          int nextShiftNumber = currentShiftNumber + 1;

          double timeUntilNextShift = (nextShiftNumber * 25.0) - timeSinceTeleopStart;

          return timeUntilNextShift;
        } else {
          return getMatchTime();
        }
      } else {
        return -1;
      }
    } else {
      return 25.0 - (getMatchTime() % 25.0);
    }
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
      return false;
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
    if (remainingTimeSeconds <= 3) return 1.0;
    else if (remainingTimeSeconds == 7) {
      return 0.5;
    } else if (remainingTimeSeconds == 13) {
      return 0.25;
    }

    return 0.0;
  }

  /**
   * Gets the current match time. In a match, this is the time until the end of the current period
   * (auto or teleop). Outside of match or in simulation, this is the time since enabled. The value
   * is always rounded to the nearest second to match FMS behavior.
   *
   * @return The current match time in seconds.
   */
  private double getMatchTime() {
    return Math.round(
        RobotBase.isReal()
            ? DriverStation.getMatchTime()
            : RobotState.isEnabled()
                ? Timer.getFPGATimestamp() - simulatedMatchStartTime
                : 0.0); // Rounded to match FMS behavior
  }
}
