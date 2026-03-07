package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.logging.LoggingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.shoot.AutoShoot;

public class AutoSegments {
  private RobotContainer robot;

  public AutoSegments(RobotContainer robot) {
    this.robot = robot;
  }

  public Command autoShoot() {
    return new AutoShoot(robot);
  }

  public Command driveToStart() {
    return robot
        .getSwerveDrive()
        .driveTo(new Pose2d(FieldPositions.START_POSITION, Rotation2d.kZero));
  }

  public Rotation2d orient() {
    double headingDegrees = robot.getSwerveDrive().getHeading().in(Degrees) % 360.0;
    if (headingDegrees < 0.0) headingDegrees += 360.0;
    if (headingDegrees > 90.0 && headingDegrees < 270.0) {
      return Rotation2d.k180deg;
    } else {
      return Rotation2d.kZero;
    }
  }

  public Command driveToMiddleAlliance() {
    return robot.getSwerveDrive().driveTo(FieldPositions.ALLIANCE_ZONE_CENTER);
  }

  public Command driveToMiddleNeutral() {
    return robot.getSwerveDrive().driveTo(FieldPositions.NEUTRAL_ZONE_CENTER);
  }

  /*
   * drive to left trench from Neutral zone
   */
  public Command driveToLeftTrenchNeutral() {
    return Commands.defer(
        () ->
            robot
                .getSwerveDrive()
                .driveTo(new Pose2d(FieldPositions.Trench.LEFT_NEUTRAL, orient())),
        robot.getSwerveDrive().useMotionSet());
  }

  /*
   * drive to left trench from alliance zone
   */
  public Command driveToLeftTrenchAlliance() {
    return Commands.defer(
        () ->
            robot
                .getSwerveDrive()
                .driveTo(new Pose2d(FieldPositions.Trench.LEFT_ALLIANCE, orient())),
        robot.getSwerveDrive().useMotionSet());
  }

  /*
   * drive to right trench from Neutral zone
   */
  public Command driveToRightTrenchNeutral() {
    return Commands.defer(
        () ->
            robot
                .getSwerveDrive()
                .driveTo(new Pose2d(FieldPositions.Trench.RIGHT_NEUTRAL, orient())),
        robot.getSwerveDrive().useMotionSet());
  }

  /*
   * drive to right trench from alliance zone
   */
  public Command driveToRightTrenchAlliance() {
    return Commands.defer(
        () ->
            robot
                .getSwerveDrive()
                .driveTo(new Pose2d(FieldPositions.Trench.RIGHT_ALLIANCE, orient())),
        robot.getSwerveDrive().useMotionSet());
  }

  public Command driveToLeftBumpNeutral() {
    return Commands.defer(
        () ->
            robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.Bump.LEFT_NEUTRAL, orient())),
        robot.getSwerveDrive().useMotionSet());
  }

  /*
   * drive to left Bump from alliance zone
   */
  public Command driveToLeftBumpAlliance() {
    return Commands.defer(
        () ->
            robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.Bump.LEFT_ALLIANCE, orient())),
        robot.getSwerveDrive().useMotionSet());
  }

  /*
   * drive to right Bump from Neutral zone
   */
  public Command driveToRightBumpNeutral() {
    return Commands.defer(
        () ->
            robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.Bump.RIGHT_NEUTRAL, orient())),
        robot.getSwerveDrive().useMotionSet());
  }

  /*
   * drive to right Bump from alliance zone
   */
  public Command driveToRightBumpAlliance() {
    return Commands.defer(
        () ->
            robot
                .getSwerveDrive()
                .driveTo(new Pose2d(FieldPositions.Bump.RIGHT_ALLIANCE, orient())),
        robot.getSwerveDrive().useMotionSet());
  }

  public Command collectFuelFromMidline() {
    return Commands.parallel(
            robot.getHopper().load(),
            robot.getIntakeRollers().intake(),
            robot.getIntakeExtension().extend())
        .withDeadline(robot.getSwerveDrive().driveTo(FieldPositions.NEUTRAL_ZONE_CENTER));
  }

  public Command collectFuelViaRightTrenchSequence() {
    return Commands.sequence(
        driveThroughRightTrenchIntoNeutral(),
        robot
            .getSwerveDrive()
            .driveTo(
                new Pose2d(
                    Inches.of(313).in(Meters),
                    Inches.of(60).in(Meters),
                    Rotation2d.fromDegrees(90)))
            .deadlineFor(
                robot.getHopper().load(),
                robot.getIntakeRollers().intake(),
                robot.getIntakeExtension().extend()),
        collectFuelFromMidline(),
        driveThroughRightTrenchIntoAlliance());
  }

  public Command collectFuelViaLeftTrenchSequence() {
    return Commands.sequence(
        driveThroughLeftTrenchIntoNeutral(),
        robot
            .getSwerveDrive()
            .driveTo(
                new Pose2d(
                    Inches.of(313).in(Meters),
                    Inches.of(260).in(Meters),
                    Rotation2d.fromDegrees(90))),
        collectFuelFromMidline(),
        driveThroughRightTrenchIntoAlliance());
  }

  /*Probably not going to be used */
  // public Command collectFuelInLeftNeutral() {
  //   return  Commands.parallel(
  //               robot
  //                   .getSwerveDrive()
  //                   .driveTo(new Pose2d(FieldPositions.NEUTRAL_ZONE_CENTER,
  // Rotation2d.fromDegrees(90))),
  //               robot.getHopper().load(),
  //               robot.getIntakeRollers().intake(),
  //               robot.getIntakeExtension().extend())
  //           .withDeadline(robot.getSwerveDrive().driveTo(new
  // Pose2d(FieldPositions.NEUTRAL_ZONE_CENTER, orient())));
  // }

  //   public Command collectFuelInRightNeutral() {
  //   return  Commands.parallel(
  //               robot
  //                   .getSwerveDrive()
  //                   .driveTo(new Pose2d(FieldPositions.NEUTRAL_ZONE_CENTER,
  // Rotation2d.fromDegrees(-90))),
  //               robot.getHopper().load(),
  //               robot.getIntakeRollers().intake(),
  //               robot.getIntakeExtension().extend())
  //           .withDeadline(robot.getSwerveDrive().driveTo(new
  // Pose2d(FieldPositions.NEUTRAL_ZONE_CENTER, orient())));
  // }

  public Command shootUntilEmpty() {
    return Commands.parallel(autoShoot(), robot.getHopper().feed())
        .until(() -> robot.getHopper().isEmpty());
  }

  public Command driveToHub() {
    return robot.getSwerveDrive().driveTo(FieldPositions.HUB_FRONT);
  }

  public Command driveThroughRightTrenchIntoAlliance() {
    return Commands.sequence(driveToRightTrenchNeutral(), driveToRightTrenchAlliance());
  }

  public Command driveThroughRightTrenchIntoNeutral() {
    return Commands.sequence(driveToRightTrenchAlliance(), driveToRightTrenchNeutral());
  }

  public Command driveThroughLeftTrenchIntoNeutral() {
    return Commands.sequence(driveToLeftTrenchAlliance(), driveToLeftTrenchNeutral());
  }

  public Command driveThroughLeftTrenchIntoAlliance() {
    return Commands.sequence(driveToLeftTrenchNeutral(), driveToLeftTrenchAlliance());
  }

  public Command driveThroughRightBumpIntoAlliance() {
    return Commands.sequence(driveToRightBumpNeutral(), driveToRightBumpAlliance());
  }

  public Command driveThroughRightBumpIntoNeutral() {
    return Commands.sequence(driveToRightBumpAlliance(), driveToRightBumpNeutral());
  }

  public Command driveThroughLeftBumpIntoNeutral() {
    return Commands.sequence(driveToLeftBumpAlliance(), driveToLeftBumpNeutral());
  }

  public Command driveThroughLeftBumpIntoAlliance() {
    return Commands.sequence(driveToLeftBumpNeutral(), driveToLeftBumpAlliance());
  }

  public Command driveToOutpost() {
    return Commands.sequence(
        robot.getSwerveDrive().driveTo(FieldPositions.OUTPOST),
        Commands.sequence(
            autoShoot().alongWith(robot.getHopper().feed()).withTimeout(2.0), shootUntilEmpty()));
  }

  public Command collectFuelCrossingViaRightTrench() {
    return Commands.sequence(
        Commands.deadline(
            driveThroughRightTrenchIntoNeutral(),
            robot.getHopper().load(),
            robot.getIntakeRollers().intake(),
            robot.getIntakeExtension().extend()),
        Commands.deadline(
            Commands.defer(
                () ->
                    robot
                        .getSwerveDrive()
                        .driveTo(
                            new Pose2d(
                                Inches.of(323).in(Meters), Inches.of(75).in(Meters), orient())),
                robot.getSwerveDrive().useMotionSet()),
            robot.getHopper().load(),
            robot.getIntakeRollers().intake()),
        Commands.deadline(
            Commands.defer(
                () ->
                    robot
                        .getSwerveDrive()
                        .driveTo(new Pose2d(FieldPositions.NEUTRAL_ZONE_CENTER, orient())),
                robot.getSwerveDrive().useMotionSet()),
            robot.getHopper().load(),
            robot.getIntakeRollers().intake()),
        Commands.parallel(
                Commands.defer(
                    () ->
                        robot
                            .getSwerveDrive()
                            .driveTo(new Pose2d(FieldPositions.ALLIANCE_ZONE_CENTER, orient())),
                    robot.getSwerveDrive().useMotionSet()),
                robot.getHopper().load(),
                robot.getIntakeRollers().intake(),
                robot.getIntakeExtension().extend())
            .withTimeout(6), /* TODO: Test and adjust this! */
        robot.getIntakeExtension().retract());
  }

  public Command collectFuelCrossingViaLeftTrench() {
    return Commands.sequence(
        Commands.deadline(
            driveThroughLeftTrenchIntoNeutral(),
            robot.getHopper().load(),
            robot.getIntakeRollers().intake(),
            robot.getIntakeExtension().extend()),
        Commands.deadline(
            Commands.defer(
                () ->
                    robot
                        .getSwerveDrive()
                        .driveTo(
                            new Pose2d(
                                Inches.of(323).in(Meters), Inches.of(250).in(Meters), orient())),
                robot.getSwerveDrive().useMotionSet()),
            robot.getHopper().load(),
            robot.getIntakeRollers().intake()),
        Commands.deadline(
            Commands.defer(
                () ->
                    robot
                        .getSwerveDrive()
                        .driveTo(new Pose2d(FieldPositions.NEUTRAL_ZONE_CENTER, orient())),
                robot.getSwerveDrive().useMotionSet()),
            robot.getHopper().load(),
            robot.getIntakeRollers().intake()),
        Commands.parallel(
                Commands.defer(
                    () ->
                        robot
                            .getSwerveDrive()
                            .driveTo(new Pose2d(FieldPositions.ALLIANCE_ZONE_CENTER, orient())),
                    robot.getSwerveDrive().useMotionSet()),
                robot.getHopper().load(),
                robot.getIntakeRollers().intake(),
                robot.getIntakeExtension().extend())
            .withTimeout(6), /* TODO: Test and adjust this! */
        robot.getIntakeExtension().retract());
  }

  public Command testAuto() {
    return Commands.sequence(
        LoggingUtil.logCommand(
            "collectFuelViaRightTrenchSequence", collectFuelViaRightTrenchSequence()),
        LoggingUtil.logCommand("shootUntilEmpty", shootUntilEmpty()),
        LoggingUtil.logCommand(
            "collectFuelViaRightTrenchSequence2", collectFuelViaRightTrenchSequence()),
        LoggingUtil.logCommand("shootUntilEmpty2", shootUntilEmpty()),
        LoggingUtil.logCommand("driveToOutpost", driveToOutpost()));
  }
}
