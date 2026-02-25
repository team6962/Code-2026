package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveFixedShooter {
  private RobotContainer robot;
  private final Translation2d HUB_POSITION =
      new Translation2d(Inches.of(182.11).in(Meters), Inches.of(158.84).in(Meters));
  public static final double HUB_RADIUS = Inches.of(40).in(Meters); // just a guess

  public DriveFixedShooter(RobotContainer robot) {
    this.robot = robot;
  }

  public Command driveToLeftPosition(double radius) {
    Translation2d shotPosition =
        HUB_POSITION.plus(new Translation2d(-radius / Math.sqrt(2.00), radius / Math.sqrt(2.00)));
    return robot
        .getSwerveDrive()
        .driveTo(new Pose2d(shotPosition, new Rotation2d(Degrees.of(-45.0))));
  }

  public Command driveToRightPosition(double radius) {
    Translation2d shotPosition =
        HUB_POSITION.plus(new Translation2d(-radius / Math.sqrt(2.00), -radius / Math.sqrt(2.00)));
    return robot
        .getSwerveDrive()
        .driveTo(new Pose2d(shotPosition, new Rotation2d(Degrees.of(45.0))));
  }

  public Command driveToClosestShotPosition(double radius) {
    Translation2d hubToRobotVector =
        robot.getSwerveDrive().getPosition2d().getTranslation().minus(HUB_POSITION);
    Translation2d shotPosition =
        HUB_POSITION.plus(hubToRobotVector.div(hubToRobotVector.getNorm()).times(radius));
    return robot
        .getSwerveDrive()
        .driveTo(
            new Pose2d(
                shotPosition, hubToRobotVector.getAngle().plus(new Rotation2d(Degrees.of(180.0)))));
  }
}
