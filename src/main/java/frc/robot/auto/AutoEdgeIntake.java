package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoEdgeIntake {
  private RobotContainer robot;

  public AutoEdgeIntake(RobotContainer robot) {
    this.robot = robot;
  }

  public Command intakeToEdgeLeft() {
    return Commands.sequence(
        robot
            .getSwerveDrive()
            .driveTo(new Pose2d(FieldPositions.Trench.LEFT_ALLIANCE, Rotation2d.kZero)),
        robot
            .getIntakeRollers()
            .intake()
            .withDeadline(robot.getSwerveDrive().driveTo(FieldPositions.LEFT_END_OF_NEUTRAL_ZONE)));
  }

  public Command intakeToEdgeRight() {
    return Commands.sequence(
        robot
            .getSwerveDrive()
            .driveTo(new Pose2d(FieldPositions.Trench.RIGHT_ALLIANCE, Rotation2d.kZero)),
        robot
            .getIntakeRollers()
            .intake()
            .withDeadline(
                robot.getSwerveDrive().driveTo(FieldPositions.RIGHT_END_OF_NEUTRAL_ZONE)));
  }
}
