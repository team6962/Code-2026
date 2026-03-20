package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoZoneDefense {
  private RobotContainer robot;

  public AutoZoneDefense(RobotContainer robot) {
    this.robot = robot;
  }

  public static enum Zone {
    OWN_ALLIANCE,
    NEUTRAL_ZONE,
    OPPOSING_ALLIANCE,
  }

  /* Determine the robot's current zone based on its X position on the field */
  public Zone getZone() {
    double robotX = robot.getSwerveDrive().getLocalization().getPosition2d().getX();

    if (robotX < FieldPositions.Trench.TRENCH_DISTANCE) {
      return Zone.OWN_ALLIANCE;
    } else if (robotX < FieldPositions.OpposingSide.OPPOSING_TRENCH_DISTANCE) {
      return Zone.NEUTRAL_ZONE;
    } else {
      return Zone.OPPOSING_ALLIANCE;
    }
  }

  /* Defend the right trench based on the robot's current zone */
  public Command defendRightTrench() {
    if (getZone() == Zone.OPPOSING_ALLIANCE) {
      return Commands.sequence(
          robot
              .getSwerveDrive()
              .driveTo(FieldPositions.OpposingSide.OPPOSING_ALLIANCE_RIGHT_TRENCH),
          robot.getSwerveDrive().lock());
    } else if (getZone() == Zone.NEUTRAL_ZONE) {
      return Commands.sequence(
          robot.getSwerveDrive().driveTo(FieldPositions.OpposingSide.OPPOSING_NEUTRAL_RIGHT_TRENCH),
          robot.getSwerveDrive().lock());
    } else {
      return Commands.none();
    }
  }

  /* Defend the right bump based on the robot's current zone */
  public Command defendRightBump() {
    if (getZone() == Zone.OPPOSING_ALLIANCE) {
      return Commands.sequence(
          robot.getSwerveDrive().driveTo(FieldPositions.OpposingSide.OPPOSING_ALLIANCE_RIGHT_BUMP),
          robot.getSwerveDrive().lock());
    } else if (getZone() == Zone.NEUTRAL_ZONE) {
      return Commands.sequence(
          robot.getSwerveDrive().driveTo(FieldPositions.OpposingSide.OPPOSING_NEUTRAL_RIGHT_BUMP),
          robot.getSwerveDrive().lock());
    } else {
      return Commands.none();
    }
  }

  /* Defend the left bump based on the robot's current zone */
  public Command defendLeftBump() {
    if (getZone() == Zone.OPPOSING_ALLIANCE) {
      return Commands.sequence(
          robot.getSwerveDrive().driveTo(FieldPositions.OpposingSide.OPPOSING_ALLIANCE_LEFT_BUMP),
          robot.getSwerveDrive().lock());
    } else if (getZone() == Zone.NEUTRAL_ZONE) {
      return Commands.sequence(
          robot.getSwerveDrive().driveTo(FieldPositions.OpposingSide.OPPOSING_NEUTRAL_LEFT_BUMP),
          robot.getSwerveDrive().lock());
    } else {
      return Commands.none();
    }
  }

  /* Defend the left trench based on the robot's current zone */
  public Command defendLeftTrench() {
    if (getZone() == Zone.OPPOSING_ALLIANCE) {
      return Commands.sequence(
          robot.getSwerveDrive().driveTo(FieldPositions.OpposingSide.OPPOSING_ALLIANCE_LEFT_TRENCH),
          robot.getSwerveDrive().lock());
    } else if (getZone() == Zone.NEUTRAL_ZONE) {
      return Commands.sequence(
          robot.getSwerveDrive().driveTo(FieldPositions.OpposingSide.OPPOSING_NEUTRAL_LEFT_TRENCH),
          robot.getSwerveDrive().lock());
    } else {
      return Commands.none();
    }
  }
}
