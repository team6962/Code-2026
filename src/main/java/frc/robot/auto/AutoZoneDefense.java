package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  /** Orient the robot to the nearest 90-degree angle based on its current heading */
  public Rotation2d orientRotation() {
    double robotRotation =
        robot.getSwerveDrive().getLocalization().getPosition2d().getRotation().getRadians();

    double orientedRotation = Math.round(robotRotation / (Math.PI / 2.0)) * (Math.PI / 2.0);

    return Rotation2d.fromRadians(orientedRotation);
  }

  /** Determine the robot's current zone based on its X position on the field */
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

  /** Defend an obstacle based on the robot's current zone and obstacle Y position */
  public Command defendObstacle(double obstacleY) {
    return Commands.defer(
        () -> {
          Pose2d targetPose =
              new Pose2d(
                  (getZone() == Zone.OPPOSING_ALLIANCE)
                      ? new Translation2d(
                          FieldPositions.OpposingSide.OPPOSING_ALLIANCE_X, obstacleY)
                      : new Translation2d(
                          FieldPositions.OpposingSide.OPPOSING_NEUTRAL_X, obstacleY),
                  orientRotation());
          return Commands.sequence(
              robot.getSwerveDrive().driveTo(targetPose), robot.getSwerveDrive().lock());
        },
        robot.getSwerveDrive().useMotionSet());
  }
}
