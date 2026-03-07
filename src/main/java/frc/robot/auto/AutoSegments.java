package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.shoot.AutoShoot;

public class AutoSegments {
  private RobotContainer robot;

  private AutoShoot autoShoot;

  public AutoSegments(RobotContainer robot) {
    this.robot = robot;
    autoShoot =
        new AutoShoot(
            robot.getSwerveDrive(),
            robot.getTurret(),
            robot.getShooterHood(),
            robot.getShooterRollers(),
            robot.getShooterFunctions(),
            () -> FieldPositions.HUB_CENTER,
            () -> Degrees.of(0),
            () -> DegreesPerSecond.of(0));
  }

  public Command driveToStart() {
    return robot
        .getSwerveDrive()
        .driveTo(new Pose2d(FieldPositions.START_POSITION, Rotation2d.kZero));
  }

  public Rotation2d orient() {
    if (robot.getSwerveDrive().getHeading().gt(Degrees.of(90))
        && robot.getSwerveDrive().getHeading().lt(Degrees.of(270))) {
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
    return robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.Trench.LEFT_NEUTRAL, orient()));
  }

  /*
   * drive to left trench from alliance zone
   */
  public Command driveToLeftTrenchAlliance() {
    return robot
        .getSwerveDrive()
        .driveTo(new Pose2d(FieldPositions.Trench.LEFT_ALLIANCE, orient()));
  }

  /*
   * drive to right trench from Neutral zone
   */
  public Command driveToRightTrenchNeutral() {
    return robot
        .getSwerveDrive()
        .driveTo(new Pose2d(FieldPositions.Trench.RIGHT_NEUTRAL, orient()));
  }

  /*
   * drive to right trench from alliance zone
   */
  public Command driveToRightTrenchAlliance() {
    return robot
        .getSwerveDrive()
        .driveTo(new Pose2d(FieldPositions.Trench.RIGHT_ALLIANCE, orient()));
  }

  public Command driveToLeftBumpNeutral() {
    return robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.Bump.LEFT_NEUTRAL, orient()));
  }

  /*
   * drive to left Bump from alliance zone
   */
  public Command driveToLeftBumpAlliance() {
    return robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.Bump.LEFT_ALLIANCE, orient()));
  }

  /*
   * drive to right Bump from Neutral zone
   */
  public Command driveToRightBumpNeutral() {
    return robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.Bump.RIGHT_NEUTRAL, orient()));
  }

  /*
   * drive to right Bump from alliance zone
   */
  public Command driveToRightBumpAlliance() {
    return robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.Bump.RIGHT_ALLIANCE, orient()));
  }

  public Command collectFuelFromMidline() {
          return  Commands.parallel(
                robot.getHopper().load(),
                robot.getIntakeRollers().intake(),
                robot.getIntakeExtension().extend())
            .withDeadline(robot
                    .getSwerveDrive()
                    .driveTo(FieldPositions.NEUTRAL_ZONE_CENTER));
  }

  public Command collectFuelViaRightTrenchSequence() {
    return Commands.sequence(
        driveThroughRightTrenchIntoNeutral(),
        robot
            .getSwerveDrive()
            .driveTo(new Pose2d(Inches.of(323).in(Meters), Inches.of(75).in(Meters),  Rotation2d.fromDegrees(90))),
        collectFuelFromMidline(),
        driveThroughRightTrenchIntoAlliance());
  }

  public Command collectFuelViaLeftTrenchSequence() {
    return Commands.sequence(
        driveThroughLeftTrenchIntoNeutral(),
        robot
            .getSwerveDrive()
            .driveTo(new Pose2d(Inches.of(323).in(Meters), Inches.of(250).in(Meters), Rotation2d.fromDegrees(90))),
        collectFuelFromMidline(),
        driveThroughRightTrenchIntoAlliance());
  }
 /*Probably not going to be used */
  // public Command collectFuelInLeftNeutral() {
  //   return  Commands.parallel(
  //               robot
  //                   .getSwerveDrive()
  //                   .driveTo(new Pose2d(FieldPositions.NEUTRAL_ZONE_CENTER, Rotation2d.fromDegrees(90))),
  //               robot.getHopper().load(),
  //               robot.getIntakeRollers().intake(),
  //               robot.getIntakeExtension().extend())
  //           .withDeadline(robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.NEUTRAL_ZONE_CENTER, orient())));
  // }

  //   public Command collectFuelInRightNeutral() {
  //   return  Commands.parallel(
  //               robot
  //                   .getSwerveDrive()
  //                   .driveTo(new Pose2d(FieldPositions.NEUTRAL_ZONE_CENTER, Rotation2d.fromDegrees(-90))),
  //               robot.getHopper().load(),
  //               robot.getIntakeRollers().intake(),
  //               robot.getIntakeExtension().extend())
  //           .withDeadline(robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.NEUTRAL_ZONE_CENTER, orient())));
  // }

  public Command shootUntilEmpty() {
    return autoShoot.until(() -> robot.getHopper().getSensors().isHopperEmpty());
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
        Commands.parallel(
            robot.getHopper().feed(),
            Commands.sequence(autoShoot.withTimeout(2.0), shootUntilEmpty())));
  }
}
