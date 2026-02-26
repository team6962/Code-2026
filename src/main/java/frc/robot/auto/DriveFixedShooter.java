package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.team6962.lib.math.AngleMath;
import com.team6962.lib.math.MeasureUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class DriveFixedShooter {
  private RobotContainer robot;
  private final Translation2d HUB_POSITION =
      new Translation2d(Inches.of(182.11).in(Meters), Inches.of(158.84).in(Meters));
  private static final double HUB_RADIUS = Inches.of(80).in(Meters);
  private static final Angle MIN_SHOOTABLE_ANGLE = Radians.of(2.018472);
  private static final Angle MAX_SHOOTABLE_ANGLE = Radians.of(Math.PI * 2 - 2.018472);

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

  public Command driveToClosestShotPosition() {
    return Commands.defer(
        () -> {
          Translation2d robotToHubVector =
              robot.getSwerveDrive().getPosition2d().getTranslation().minus(HUB_POSITION);

          Angle robotToHubAngle = robotToHubVector.getAngle().getMeasure();

          robotToHubAngle = AngleMath.toDiscrete(robotToHubAngle);

          if (robotToHubAngle.lt(Radians.of(0))) {
            robotToHubAngle = robotToHubAngle.plus(Radians.of(2 * Math.PI));
          }

          robotToHubAngle =
              MeasureUtil.clamp(robotToHubAngle, MIN_SHOOTABLE_ANGLE, MAX_SHOOTABLE_ANGLE);

          Rotation2d robotToHubRotation = new Rotation2d(robotToHubAngle);
          Translation2d shootingPoseRelativeToHub =
              new Translation2d(HUB_RADIUS, 0).rotateBy(robotToHubRotation);

          Translation2d shotPosition = HUB_POSITION.plus(shootingPoseRelativeToHub);

          return robot
              .getSwerveDrive()
              .driveTo(
                  new Pose2d(shotPosition, robotToHubRotation.plus(Rotation2d.fromDegrees(45))));
        },
        robot.getSwerveDrive().useMotionSet());
  }
}
