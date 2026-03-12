package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import java.util.Set;

public class NeutralIntake {
  private static Pose2d INITIAL_POSE = new Pose2d(7.817, 1.219, Rotation2d.fromDegrees(90));
  private static Distance HUB_Y = Meters.of(4.03463125);

  private static enum Side {
    LEFT,
    RIGHT
  };

  private RobotContainer robot;

  public NeutralIntake(RobotContainer robot) {
    this.robot = robot;
  }

  /**
   * Returns a Pose2d representing the position the robot should drive to in order to intake fuel
   * from the neutral zone, based on the provided side and distance into the mass of fuel.
   *
   * @param side the side of the neutral zone to intake from (LEFT or RIGHT)
   * @param distance the distance into the mass of fuel to drive (e.g., 0 for the edge, increasing
   *     as you go further in)
   * @return the Pose2d representing the target intake position
   */
  private Pose2d getIntakePose(Side side, Distance distance) {
    Pose2d rightPose =
        new Pose2d(
            INITIAL_POSE.getX(),
            INITIAL_POSE.getY() + distance.in(Meters),
            INITIAL_POSE.getRotation());

    if (side == Side.RIGHT) {
      return rightPose;
    } else {
      return new Pose2d(
          rightPose.getMeasureX(),
          HUB_Y.times(2).minus(rightPose.getMeasureY()),
          rightPose.getRotation().times(-1));
    }
  }

  public Command intake(
      Side side,
      Distance initialDistance,
      Distance finalDistance,
      LinearVelocity initialVelocity,
      LinearVelocity finalVelocity) {
    double velocitySign = (side == Side.RIGHT) ? 1 : -1;

    return robot
        .getSwerveDrive()
        .driveTo(
            getIntakePose(side, initialDistance),
            new ChassisSpeeds(velocitySign * initialVelocity.in(MetersPerSecond), 0, 0))
        .andThen(
            robot
                .getSwerveDrive()
                .driveTo(
                    getIntakePose(side, finalDistance),
                    new ChassisSpeeds(velocitySign * finalVelocity.in(MetersPerSecond), 0, 0)))
        .deadlineFor(robot.getIntakeRollers().intake(), robot.getIntakeExtension().extend());
  }

  public Command intakeLeft(
      Distance initialDistance,
      Distance finalDistance,
      LinearVelocity initialVelocity,
      LinearVelocity finalVelocity) {
    return intake(Side.LEFT, initialDistance, finalDistance, initialVelocity, finalVelocity);
  }

  public Command intakeRight(
      Distance initialDistance,
      Distance finalDistance,
      LinearVelocity initialVelocity,
      LinearVelocity finalVelocity) {
    return intake(Side.RIGHT, initialDistance, finalDistance, initialVelocity, finalVelocity);
  }

  public Command intakeLeft(Distance distance) {
    return intakeLeft(Meters.of(0), distance, MetersPerSecond.of(1), MetersPerSecond.of(0));
  }

  public Command intakeRight(Distance distance) {
    return intakeRight(Meters.of(0), distance, MetersPerSecond.of(1), MetersPerSecond.of(0));
  }

  public Command intake(
      Distance initialDistance,
      Distance finalDistance,
      LinearVelocity initialVelocity,
      LinearVelocity finalVelocity) {
    return Commands.defer(
        () -> {
          if (robot.getSwerveDrive().getPosition2d().getMeasureY().gt(HUB_Y)) {
            return intakeLeft(initialDistance, finalDistance, initialVelocity, finalVelocity);
          } else {
            return intakeRight(initialDistance, finalDistance, initialVelocity, finalVelocity);
          }
        },
        Set.of(
            robot.getSwerveDrive().useTranslation(),
            robot.getSwerveDrive().useRotation(),
            robot.getIntakeExtension(),
            robot.getIntakeRollers()));
  }

  public Command intake(Distance distance) {
    return intake(distance, distance, MetersPerSecond.of(1), MetersPerSecond.of(0));
  }

  public Command intake(Distance initialDistance, Distance finalDistance) {
    return intake(initialDistance, finalDistance, MetersPerSecond.of(1), MetersPerSecond.of(0));
  }
}
