package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.team6962.lib.math.AngleMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

/** Handles driving through the trenches autonomously. */
public class TrenchDriving {
  public static Distance OBSTACLES_CENTER_X = Meters.of(4.67);
  public static Distance OBSTACLES_CENTER_OPPOSITE_X = Meters.of( 11.870988);
  private static Distance RIGHT_TRENCH_CENTER_Y = Inches.of(24.92);
  private static Distance HUB_Y = Meters.of(4.03463125);
  private static Distance CENTER_FIELD_X = Meters.of(8.270494);
  private static Distance TRENCH_INITIAL_X = Inches.of(48);
  private static Distance TRENCH_FINAL_X = Inches.of(48);
  private static Distance NEAR_TRENCH_ENTERANCE_DISTANCE = Inches.of(60);
  private static LinearVelocity TRENCH_VELOCITY = MetersPerSecond.of(1.5);
  private static Distance TRENCH_LINEAR_TOLERANCE = Inches.of(10);
  private static Angle TRENCH_ANGULAR_TOLERANCE = Degrees.of(20);

  private RobotContainer robot;

  public static enum Trench {
    LEFT,
    RIGHT,
    LEFT_OPPOSITE,
    RIGHT_OPPOSITE
  }

  /**
   * Creates a new AutoTrench command provider.
   *
   * @param robot The RobotContainer instance.
   */
  public TrenchDriving(RobotContainer robot) {
    this.robot = robot;
  }

  //Checks if the robot is on the left side of the field
  public Boolean isOnLeftSide() {
    Distance robotX = Meters.of(robot.getSwerveDrive().getPosition2d().getX());
    if (robotX.lt(CENTER_FIELD_X)) {
      return true;
    } else {
      return false;
    }
  }

  public Trench getNearestTrench() {
    Distance robotY = Meters.of(robot.getSwerveDrive().getPosition2d().getY());
    Distance robotX = Meters.of(robot.getSwerveDrive().getPosition2d().getX());

    if (robotY.lt(HUB_Y) && robotX.lt(CENTER_FIELD_X)) {
      return Trench.RIGHT;
    } else if (robotY.gt(HUB_Y) && robotX.lt(CENTER_FIELD_X)){
      return Trench.LEFT;
    } else if (robotY.lt(HUB_Y) && robotX.gt(CENTER_FIELD_X)) {
      return Trench.RIGHT_OPPOSITE;
    } else {
      return Trench.LEFT_OPPOSITE;
    }
  }


  public Command getNearestTrenchDriveCommand() {
    Distance robotY = Meters.of(robot.getSwerveDrive().getPosition2d().getY());
    Distance robotX = Meters.of(robot.getSwerveDrive().getPosition2d().getX());

    Trench nearestTrench; 

    if (robotY.lt(HUB_Y) && robotX.lt(CENTER_FIELD_X)) {
      nearestTrench = Trench.RIGHT;
    } else if (robotY.gt(HUB_Y) && robotX.lt(CENTER_FIELD_X)){
      nearestTrench = Trench.LEFT;
    } else if (robotY.lt(HUB_Y) && robotX.gt(CENTER_FIELD_X)) {
      nearestTrench = Trench.RIGHT_OPPOSITE;
    } else {
      nearestTrench = Trench.LEFT_OPPOSITE;
    }

    if (nearestTrench == Trench.RIGHT || nearestTrench == Trench.LEFT) {
      if (robotX.lt(OBSTACLES_CENTER_X)) {
        return driveToNeutral(nearestTrench);
      } else {
        return driveToAlliance(nearestTrench);
      }
    } else {
        if (robotX.lt(OBSTACLES_CENTER_OPPOSITE_X)) {
          return driveToAlliance(nearestTrench);
        } else{
          return driveToNeutral(nearestTrench);
        }
    }
  }

  private Distance getNearestTrenchCenterY(Trench trench) {
    if (trench == null) {
      trench = getNearestTrench();
    }

    if (trench == Trench.RIGHT) {
      return RIGHT_TRENCH_CENTER_Y;
    } else if (trench == Trench.LEFT) {
      return HUB_Y.times(2).minus(RIGHT_TRENCH_CENTER_Y);
    } else if (trench == Trench.RIGHT_OPPOSITE) {
      return RIGHT_TRENCH_CENTER_Y;
    } else {
      return HUB_Y.times(2).minus(RIGHT_TRENCH_CENTER_Y);
    }
  }

  private Rotation2d getTrenchRobotOrientation(Rotation2d overrideOrientation) {
    if (overrideOrientation != null) {
      return overrideOrientation;
    }

    Angle robotHeading = AngleMath.toDiscrete(robot.getSwerveDrive().getHeading());

    if (robotHeading.abs(Degrees) < 90) {
      return Rotation2d.kZero;
    } else {
      return Rotation2d.k180deg;
    }
  }

  private Pose2d getInitialAlliancePose(Trench overrideTrench, Rotation2d overrideOrientation) {
    if (isOnLeftSide())
      return new Pose2d(
        OBSTACLES_CENTER_X.minus(TRENCH_INITIAL_X),
        getNearestTrenchCenterY(overrideTrench),
        getTrenchRobotOrientation(overrideOrientation));
    else 
      return new Pose2d(
        OBSTACLES_CENTER_OPPOSITE_X.plus(TRENCH_INITIAL_X),
        getNearestTrenchCenterY(overrideTrench),
        getTrenchRobotOrientation(overrideOrientation));
  }

  private Pose2d getFinalAlliancePose(Trench overrideTrench, Rotation2d overrideOrientation) {
    if (isOnLeftSide()) {
      return new Pose2d(
        OBSTACLES_CENTER_X.minus(TRENCH_FINAL_X),
        getNearestTrenchCenterY(overrideTrench),
        getTrenchRobotOrientation(overrideOrientation));
    } else
      return new Pose2d(
        OBSTACLES_CENTER_OPPOSITE_X.plus(TRENCH_FINAL_X),
        getNearestTrenchCenterY(overrideTrench),
        getTrenchRobotOrientation(overrideOrientation));
  }

  private Pose2d getInitialNeutralPose(Trench overrideTrench, Rotation2d overrideOrientation) {
    if (isOnLeftSide()) {
      return new Pose2d(
          OBSTACLES_CENTER_X.plus(TRENCH_INITIAL_X),
          getNearestTrenchCenterY(overrideTrench),
          getTrenchRobotOrientation(overrideOrientation));
    } else {
      return new Pose2d(
          OBSTACLES_CENTER_OPPOSITE_X.minus(TRENCH_INITIAL_X),
          getNearestTrenchCenterY(overrideTrench),
          getTrenchRobotOrientation(overrideOrientation));
    }
  }

  private Pose2d getFinalNeutralPose(Trench overrideTrench, Rotation2d overrideOrientation) {
    if (isOnLeftSide()) {
      return new Pose2d(
          OBSTACLES_CENTER_X.plus(TRENCH_FINAL_X),
          getNearestTrenchCenterY(overrideTrench),
          getTrenchRobotOrientation(overrideOrientation));
    } else {
      return new Pose2d(
          OBSTACLES_CENTER_OPPOSITE_X.minus(TRENCH_FINAL_X),
          getNearestTrenchCenterY(overrideTrench),
          getTrenchRobotOrientation(overrideOrientation));
    }
  }

  private boolean isInTrench(double x) {
    return x > OBSTACLES_CENTER_X.minus(NEAR_TRENCH_ENTERANCE_DISTANCE).in(Meters)
        && x < OBSTACLES_CENTER_X.plus(NEAR_TRENCH_ENTERANCE_DISTANCE).in(Meters);
  }

  /**
   * Creates a command that drives from the alliance zone to the neutral zone through the trench.
   *
   * @param initialVelocity The initial velocity of the robot.
   * @param finalVelocity The final velocity of the robot.
   * @param trench The trench to drive through, or {@code null} to automatically determine the
   *     trench based on the robot's position.
   * @param orientation The orientation of the robot while driving through the trench, or {@code
   *     null} to automatically determine the orientation based on the robot's heading.
   * @return A command that drives from the alliance zone to the neutral zone through the trench.
   */
  public Command driveToNeutral(
      LinearVelocity initialVelocity,
      LinearVelocity finalVelocity,
      Trench trench,
      Rotation2d orientation) {
    return Commands.defer(
        () ->
            Commands.sequence(
                robot
                    .getSwerveDrive()
                    .driveTo(
                        getInitialAlliancePose(trench, orientation),
                        new ChassisSpeeds(initialVelocity.in(MetersPerSecond), 0, 0))
                    .onlyIf(() -> !isInTrench(robot.getSwerveDrive().getPosition2d().getX())),
                robot
                    .getSwerveDrive()
                    .driveTo(getInitialAlliancePose(trench, orientation))
                    .repeatedly()
                    .onlyWhile(
                        () ->
                            !robot
                                .getSwerveDrive()
                                .isNear(
                                    getInitialAlliancePose(trench, orientation),
                                    TRENCH_LINEAR_TOLERANCE,
                                    TRENCH_ANGULAR_TOLERANCE))
                    .onlyIf(() -> !isInTrench(robot.getSwerveDrive().getPosition2d().getX())),
                robot
                    .getSwerveDrive()
                    .driveTo(
                        getFinalNeutralPose(trench, orientation),
                        new ChassisSpeeds(finalVelocity.in(MetersPerSecond), 0, 0))),
        robot.getSwerveDrive().useMotionSet());
  }

  /**
   * Creates a command that drives from the alliance zone to the neutral zone through the trench.
   *
   * @param initialVelocity The initial velocity of the robot.
   * @param finalVelocity The final velocity of the robot.
   * @return A command that drives from the alliance zone to the neutral zone through the trench at
   *     the specified initial and final velocities, automatically determining the robot's
   *     orientation based on its heading.
   */
  public Command driveToNeutral(LinearVelocity initialVelocity, LinearVelocity finalVelocity) {
    return driveToNeutral(initialVelocity, finalVelocity, null, null);
  }

  /**
   * Creates a command that drives from the neutral zone to the alliance zone through the trench.
   *
   * @param initialVelocity The initial velocity of the robot.
   * @param finalVelocity The final velocity of the robot.
   * @param trench The trench to drive through, or {@code null} to automatically determine the
   *     trench based on the robot's position.
   * @param orientation The orientation of the robot while driving through the trench, or {@code
   *     null} to automatically determine the orientation based on the robot's heading.
   * @return A command that drives from the neutral zone to the alliance zone through the trench.
   */
  public Command driveToAlliance(
      LinearVelocity initialVelocity,
      LinearVelocity finalVelocity,
      Trench trench,
      Rotation2d orientation) {
    return Commands.defer(
        () ->
            Commands.sequence(
                robot
                    .getSwerveDrive()
                    .driveTo(
                        getInitialNeutralPose(trench, orientation),
                        new ChassisSpeeds(-initialVelocity.in(MetersPerSecond), 0, 0)),
                robot
                    .getSwerveDrive()
                    .driveTo(getInitialNeutralPose(trench, orientation))
                    .repeatedly()
                    .onlyWhile(
                        () ->
                            !robot
                                .getSwerveDrive()
                                .isNear(
                                    getInitialNeutralPose(trench, orientation),
                                    TRENCH_LINEAR_TOLERANCE,
                                    TRENCH_ANGULAR_TOLERANCE)),
                robot
                    .getSwerveDrive()
                    .driveTo(
                        getFinalAlliancePose(trench, orientation),
                        new ChassisSpeeds(-finalVelocity.in(MetersPerSecond), 0, 0))),
        robot.getSwerveDrive().useMotionSet());
  }

  /**
   * Creates a command that drives from the neutral zone to the alliance zone through the trench.
   *
   * @param initialVelocity The initial velocity of the robot.
   * @param finalVelocity The final velocity of the robot.
   * @return A command that drives from the neutral zone to the alliance zone through the trench at
   *     the specified initial and final velocities, automatically determining the robot's
   *     orientation based on its heading.
   */
  public Command driveToAlliance(LinearVelocity initialVelocity, LinearVelocity finalVelocity) {
    return driveToAlliance(initialVelocity, finalVelocity, null, null);
  }

  /**
   * Creates a command that drives from the alliance zone to the neutral zone through the trench at
   * a default velocity.
   *
   * @return A command that drives from the alliance zone to the neutral zone through the trench at
   *     a default velocity.
   */
  public Command driveToNeutral() {
    return driveToNeutral(TRENCH_VELOCITY, TRENCH_VELOCITY);
  }

  /**
   * Creates a command that drives from the neutral zone to the alliance zone through the trench at
   * a default velocity.
   *
   * @return A command that drives from the neutral zone to the alliance zone through the trench at
   *     a default velocity.
   */
  public Command driveToAlliance() {
    return driveToAlliance(TRENCH_VELOCITY, TRENCH_VELOCITY);
  }

  /**
   * Creates a command that drives from the alliance zone to the neutral zone through the trench at
   * a default velocity and given orientation.
   *
   * @param orientation The orientation of the robot while driving through the trench, or {@code
   *     null} to automatically determine the orientation based on the robot's heading.
   * @return A command that drives from the alliance zone to the neutral zone through the trench at
   *     a default velocity and given orientation.
   */
  public Command driveToNeutral(Rotation2d orientation) {
    return driveToNeutral(TRENCH_VELOCITY, TRENCH_VELOCITY, null, orientation);
  }

  /**
   * Creates a command that drives from the neutral zone to the alliance zone through the trench at
   * a default velocity and given orientation.
   *
   * @param orientation The orientation of the robot while driving through the trench, or {@code
   *     null} to automatically determine the orientation based on the robot's heading.
   * @return A command that drives from the neutral zone to the alliance zone through the trench at
   *     a default velocity and given orientation.
   */
  public Command driveToAlliance(Rotation2d orientation) {
    return driveToAlliance(TRENCH_VELOCITY, TRENCH_VELOCITY, null, orientation);
  }

  /**
   * Creates a command that drives through the specified trench to the neutral zone, automatically
   * determining the robot's orientation based on its heading.
   */
  public Command driveToNeutral(Trench trench) {
    return driveToNeutral(TRENCH_VELOCITY, TRENCH_VELOCITY, trench, null);
  }

  /**
   * Creates a command that drives through the specified trench to the alliance zone, automatically
   * determining the robot's orientation based on its heading.
   */
  public Command driveToAlliance(Trench trench) {
    return driveToAlliance(TRENCH_VELOCITY, TRENCH_VELOCITY, trench, null);
  }

  /**
   * Creates a command that drives through the specified trench to the neutral zone at a default
   * velocity and given orientation.
   *
   * @param trench The trench to drive through.
   * @param orientation The orientation of the robot while driving through the trench, or {@code
   *     null} to automatically determine the orientation based on the robot's heading.
   * @return A command that drives through the specified trench to the neutral zone at a default
   *     velocity and given orientation.
   */
  public Command driveToNeutral(Trench trench, Rotation2d orientation) {
    return driveToNeutral(TRENCH_VELOCITY, TRENCH_VELOCITY, trench, orientation);
  }

  /**
   * Creates a command that drives through the specified trench to the alliance zone at a default
   * velocity and given orientation.
   *
   * @param trench The trench to drive through.
   * @param orientation The orientation of the robot while driving through the trench, or {@code
   *     null} to automatically determine the orientation based on the robot's heading.
   * @return A command that drives through the specified trench to the alliance zone at a default
   *     velocity and given orientation.
   */
  public Command driveToAlliance(Trench trench, Rotation2d orientation) {
    return driveToAlliance(TRENCH_VELOCITY, TRENCH_VELOCITY, trench, orientation);
  }
}
