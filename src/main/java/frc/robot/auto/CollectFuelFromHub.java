package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class CollectFuelFromHub {
  private RobotContainer robot;
  private TrenchDriving trenchDriving;

  public CollectFuelFromHub(RobotContainer robot, TrenchDriving trenchDriving) {
    this.robot = robot;
    this.trenchDriving = trenchDriving;
  }

  public Command scoopingUpFuelRight() {
    return Commands.sequence(
        trenchDriving.driveToNeutral(),
        Commands.sequence(
            robot
                .getSwerveDrive()
                .driveTo(new Pose2d(FieldPositions.HUB_BACK, Rotation2d.fromDegrees(-150))),
            Commands.parallel(robot.getIntakeRollers().intake(), robot.getHopper().load())
                .withDeadline(robot.getSwerveDrive().driveTo(FieldPositions.Trench.RIGHT_NEUTRAL))),
        trenchDriving.driveToAlliance());
  }

  public Command scoopingUpFuelLeft() {
    return Commands.sequence(
        trenchDriving.driveToNeutral(),
        Commands.sequence(
            robot
                .getSwerveDrive()
                .driveTo(new Pose2d(FieldPositions.HUB_BACK, Rotation2d.fromDegrees(150))),
            Commands.parallel(robot.getIntakeRollers().intake(), robot.getHopper().load())
                .withDeadline(robot.getSwerveDrive().driveTo(FieldPositions.Trench.LEFT_NEUTRAL))),
        trenchDriving.driveToAlliance());
  }
}
