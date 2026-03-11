package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class CollectFuelFromHub {
  private RobotContainer robot;

  public CollectFuelFromHub(RobotContainer robot) {
    this.robot = robot;
  }

  public Command intakeBehindHubRight() {
    return Commands.sequence(
        robot
            .getSwerveDrive()
            .driveTo(new Pose2d(FieldPositions.HUB_BACK, Rotation2d.fromDegrees(-195))),
            Commands.parallel(robot.getIntakeRollers().intake(), robot.getHopper().load())
            .withDeadline(robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.Trench.RIGHT_NEUTRAL, Rotation2d.fromDegrees(-150)))));
  }

  public Command intakeBehindHubLeft() {
    return Commands.sequence(
        robot
            .getSwerveDrive()
            .driveTo(new Pose2d(FieldPositions.HUB_BACK, Rotation2d.fromDegrees(195))),
            Commands.parallel(robot.getIntakeRollers().intake(), robot.getHopper().load())
            .withDeadline(robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.Trench.LEFT_NEUTRAL, Rotation2d.fromDegrees(150))))
        );
  }
}
