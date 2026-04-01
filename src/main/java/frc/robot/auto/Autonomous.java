package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.shoot.AutoShoot;
import java.util.function.Supplier;

/** Contains autonomous command sequences that can be selected on the dashboard. */
public class Autonomous {
  private RobotContainer robot;
  public final TrenchDriving trench;
  public final NeutralIntake neutralIntake;
  public final ShootFuel shootFuel;
  public final AutoDepot autoDepot;
  public final AutoOutpost autoOutpost;
  public final AutoEdgeIntake autoEdgeIntake;
  public final CollectFuelFromHub collectFuelFromHub;

  public Autonomous(RobotContainer robot) {
    this.robot = robot;
    this.trench = new TrenchDriving(robot);
    this.neutralIntake = new NeutralIntake(robot);
    this.shootFuel = new ShootFuel(robot);
    this.autoDepot = new AutoDepot(robot);
    this.autoOutpost = new AutoOutpost(robot, shootFuel);
    this.autoEdgeIntake = new AutoEdgeIntake(robot);
    this.collectFuelFromHub = new CollectFuelFromHub(robot);
  }

  public Command trenchCheck(
      Pose2d targetPose,
      Supplier<Command> needsTrenchCheck,
      Supplier<Command> doesntNeedTrenchCheck) {
    return Commands.defer(
        () -> {
          double robotY = robot.getSwerveDrive().getPosition2d().getY();
          Rotation2d robotRotation = robot.getSwerveDrive().getPosition2d().getRotation();
          if ((robotY < Inches.of(15.97).in(Meters)
                  || (robotY > Inches.of(35.97).in(Meters)
                      && robotY < Inches.of(158.84).in(Meters)))
              || (robotY > Inches.of(302.31).in(Meters)
                  || (robotY < Inches.of(282.31).in(Meters)
                      && robotY > Inches.of(158.84).in(Meters)))
              || (Math.abs(robotRotation.getDegrees() - targetPose.getRotation().getDegrees())
                  > 15)) {
            return robot.getSwerveDrive().driveTo(targetPose).andThen(needsTrenchCheck.get());
          } else {
            return doesntNeedTrenchCheck.get();
          }
        },
        robot.getSwerveDrive().useMotionSet());
  }

  private static Pose2d LEFT_START_POSE =
      new Pose2d(4.396968364715576, 7.652250289916992, new Rotation2d());

  private Command singleNeutralCycle(boolean rightSide) {
    robot.getSwerveDrive().loadChoreoPath("left_neutral.0");
    robot.getSwerveDrive().loadChoreoPath("left_neutral.1");
    robot.getSwerveDrive().loadChoreoPath("left_neutral.2");

    return Commands.sequence(
        Commands.runOnce(
            () ->
                robot
                    .getSwerveDrive()
                    .getLocalization()
                    .resetPosition(mirrorPose(LEFT_START_POSE, rightSide))),
        robot
            .getSwerveDrive()
            .followPath("left_neutral.0", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
        robot
            .getSwerveDrive()
            .followPath("left_neutral.1", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
        robot.getSwerveDrive().followPath("left_neutral.2", rightSide),
        shootFuel.shoot());
  }

  private Command doubleNeutralCycle(boolean rightSide) {
    robot.getSwerveDrive().loadChoreoPath("left_neutral.0");
    robot.getSwerveDrive().loadChoreoPath("left_neutral.1");
    robot.getSwerveDrive().loadChoreoPath("left_neutral.2");
    robot.getSwerveDrive().loadChoreoPath("left_neutral.3");
    robot.getSwerveDrive().loadChoreoPath("left_neutral.4");
    robot.getSwerveDrive().loadChoreoPath("left_neutral.5");
    robot.getSwerveDrive().loadChoreoPath("left_neutral.6");

    return Commands.sequence(
        Commands.runOnce(
            () ->
                robot
                    .getSwerveDrive()
                    .getLocalization()
                    .resetPosition(mirrorPose(LEFT_START_POSE, rightSide))),
        robot
            .getSwerveDrive()
            .followPath("left_neutral.0", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
        robot
            .getSwerveDrive()
            .followPath("left_neutral.1", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
        robot.getSwerveDrive().followPath("left_neutral.2", rightSide),
        shootFuel.shootAllFuelStationary().withTimeout(20.0 - 13.2),
        robot
            .getSwerveDrive()
            .followPath("left_neutral.3", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
        robot
            .getSwerveDrive()
            .followPath("left_neutral.4", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
        robot
            .getSwerveDrive()
            .followPath("left_neutral.5", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
        robot.getSwerveDrive().followPath("left_neutral.6", rightSide),
        shootFuel.shoot());
  }

  public Command bump(boolean rightSide) {
    robot.getSwerveDrive().loadChoreoPath("left_neutral_bump.0");
    robot.getSwerveDrive().loadChoreoPath("left_neutral_bump.1");
    robot.getSwerveDrive().loadChoreoPath("left_neutral_bump.2");
    robot.getSwerveDrive().loadChoreoPath("left_neutral_bump.3");
    robot.getSwerveDrive().loadChoreoPath("left_neutral_bump.4");
    robot.getSwerveDrive().loadChoreoPath("left_neutral_bump.5");

    return Commands.sequence(
            Commands.runOnce(
                () ->
                    robot
                        .getSwerveDrive()
                        .getLocalization()
                        .resetPosition(mirrorPose(LEFT_START_POSE, rightSide))),
            robot
                .getSwerveDrive()
                .followPath("left_neutral_bump.0", rightSide)
                .deadlineFor(
                    robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
            robot
                .getSwerveDrive()
                .followPath("left_neutral_bump.1", rightSide)
                .deadlineFor(
                    robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
            robot.getSwerveDrive().followPath("left_neutral_bump.2", rightSide),
            shootFuel.shootAllFuelStationary().withTimeout(5),
            robot
                .getSwerveDrive()
                .followPath("left_neutral_bump.3", rightSide)
                .deadlineFor(
                    robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
            robot
                .getSwerveDrive()
                .followPath("left_neutral_bump.4", rightSide)
                .deadlineFor(
                    robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
            robot.getSwerveDrive().followPath("left_neutral_bump.5", rightSide),
            shootFuel.shoot())
        .withTimeout(20);
  }

  // Auto starting at the Mid-Hub position, moving backwards and then shooting.
  public Command moveBackwardAndShoot() {
    AutoShoot autoShoot = new AutoShoot(robot);
    return Commands.sequence(
        robot.getSwerveDrive().driveTo(FieldPositions.HUB_FURTHER_FRONT),
        Commands.parallel(
            shootFuel.shoot().onlyWhile(autoShoot.isReadyToShoot()).repeatedly(), autoShoot));
  }

  public Command leftSingleNeutralCycle() {
    return singleNeutralCycle(false);
  }

  public Command rightSingleNeutralCycle() {
    return singleNeutralCycle(true);
  }

  public Command leftDoubleNeutralCycle() {
    return doubleNeutralCycle(false);
  }

  public Command rightDoubleNeutralCycle() {
    return doubleNeutralCycle(true);
  }

  public Command preload() {
    return shootFuel.shoot();
  }

  private static Pose2d mirrorPose(Pose2d pose, boolean mirrored) {
    if (!mirrored) {
      return pose;
    }

    return new Pose2d(pose.getX(), 8.0692625 - pose.getY(), pose.getRotation().unaryMinus());
  }
}
