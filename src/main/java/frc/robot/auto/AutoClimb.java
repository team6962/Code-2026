package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
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

public class AutoClimb {
  /**
   * The pose where the robot is at the end of the pole, but the climb mechanism is not quite over
   * the end. This is the climb position that is closest to the blue outpost.
   */
  public static Pose2d END_OF_POLE_POSE =
      new Pose2d(1.117950, 2.823665, Rotation2d.fromDegrees(-90));

  /** The center of the blue tower in the Y direction, in meters. */
  public static double CENTER_OF_TOWER = 3.745706;

  /** The velocity at which the robot should be moving at when it reaches the prepare climb pose. */
  public static LinearVelocity ENTER_VELOCITY = MetersPerSecond.of(1.0);

  /**
   * The distance away from the pole that the robot should be when it reaches the prepare climb
   * pose.
   */
  public static Distance ENTER_DISTANCE = Meters.of(0.3);

  /**
   * The distance away from the pole that the robot should be when it reaches the align climb pose.
   */
  public static Distance ALIGN_DISTANCE = Meters.of(0.1);

  /**
   * The distance that the robot should be from the pole when it is considered to have left the
   * pole.
   */
  public static Distance LEAVE_DISTANCE = Meters.of(0.1);

  /**
   * The velocity at which the robot should be moving at when it is considered to have left the
   * pole.
   */
  public static LinearVelocity LEAVE_VELOCITY = MetersPerSecond.of(0.1);

  /** The distance that the robot should move along the pole to be fully over the pole. */
  public static Distance CLIMB_DISTANCE = Meters.of(0.131467);

  public static enum ClimbSide {
    LEFT,
    RIGHT
  }

  /**
   * Gets the pose for climbing based on the side of the pole and the distance along the pole. The
   * distance along the pole is how far the robot has moved along the pole. A distance of 0 means
   * the robot is at the end of the pole, and a positive distance means the climb mechanism is
   * hooked onto the pole.
   *
   * @param side the side of the pole the robot is climbing on. The right side is the side closest
   *     to the blue outpost, and the left side is the side closest to the blue depot.
   * @param distanceAlongPole the distance in meters along the pole that the robot has climbed. A
   *     distance of 0 means the robot is at the end of the pole, and a positive distance means the
   *     climb mechanism is hooked onto the pole.
   * @return the climb pose on the given side of the field at the given distance along the pole.
   */
  public static Pose2d getClimbPose(ClimbSide side, Distance distanceAlongPole) {
    Pose2d pose = END_OF_POLE_POSE;

    pose = new Pose2d(pose.getX(), pose.getY() + distanceAlongPole.in(Meters), pose.getRotation());

    if (side == ClimbSide.LEFT) {
      pose =
          new Pose2d(
              pose.getX(), 2 * CENTER_OF_TOWER - pose.getY(), pose.getRotation().unaryMinus());
    }

    return pose;
  }

  private RobotContainer robot;

  public AutoClimb(RobotContainer robot) {
    this.robot = robot;
  }

  public Command driveToClimbPose(
      ClimbSide side, Distance distanceAlongPole, LinearVelocity velocity) {
    double velocityMetersPerSecond = velocity.in(MetersPerSecond);

    return robot
        .getSwerveDrive()
        .driveTo(
            getClimbPose(side, distanceAlongPole),
            new ChassisSpeeds(
                0, side == ClimbSide.LEFT ? -velocityMetersPerSecond : velocityMetersPerSecond, 0));
  }

  public Command driveToPrepareClimb(ClimbSide side) {
    return driveToClimbPose(side, ENTER_DISTANCE.unaryMinus(), ENTER_VELOCITY);
  }

  public Command driveToAlignClimb(ClimbSide side) {
    return driveToClimbPose(side, ALIGN_DISTANCE.unaryMinus(), MetersPerSecond.of(0));
  }

  public Command driveOverPole(ClimbSide side) {
    return driveToClimbPose(side, CLIMB_DISTANCE, MetersPerSecond.of(0));
  }

  public Command driveToUnclimbPose(ClimbSide side) {
    return driveToClimbPose(side, LEAVE_DISTANCE.unaryMinus(), LEAVE_VELOCITY.unaryMinus());
  }

  public Command climb() {
    return Commands.defer(
        () -> {
          ClimbSide side =
              robot.getSwerveDrive().getPosition2d().getY() > CENTER_OF_TOWER
                  ? ClimbSide.LEFT
                  : ClimbSide.RIGHT;
          return Commands.sequence(
              Commands.deadline(driveToPrepareClimb(side), robot.getClimb().elevate()),
              Commands.deadline(
                  robot.getClimb().elevate(),
                  driveToAlignClimb(side)
                      .repeatedly()
                      .until(
                          () ->
                              robot
                                  .getSwerveDrive()
                                  .isNear(
                                      getClimbPose(side, ALIGN_DISTANCE.unaryMinus()),
                                      Inches.of(0.125),
                                      Degrees.of(2)))),
              driveToAlignClimb(side)
                  .repeatedly()
                  .until(
                      () ->
                          robot
                              .getSwerveDrive()
                              .isNear(
                                  getClimbPose(side, ALIGN_DISTANCE.unaryMinus()),
                                  Inches.of(0.125),
                                  Degrees.of(2))),
              driveOverPole(side),
              robot.getClimb().pullUp());
        },
        Set.of(
            robot.getClimb(),
            robot.getSwerveDrive().useTranslation(),
            robot.getSwerveDrive().useRotation()));
  }

  public Command unclimb() {
    return Commands.defer(
        () -> {
          ClimbSide side =
              robot.getSwerveDrive().getPosition2d().getY() > CENTER_OF_TOWER
                  ? ClimbSide.LEFT
                  : ClimbSide.RIGHT;

          return Commands.sequence(robot.getClimb().elevate(), driveToUnclimbPose(side));
        },
        Set.of(
            robot.getClimb(),
            robot.getSwerveDrive().useTranslation(),
            robot.getSwerveDrive().useRotation()));
  }
}
