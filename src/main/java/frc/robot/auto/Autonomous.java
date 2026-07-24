package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
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
  private static Pose2d LEFT_START_POSE_ROTATED =
      new Pose2d(4.396968364715576, 7.652250289916992, new Rotation2d(Degrees.of(-90)));

  private Command singleNeutralCycle(boolean rightSide) {
    String pathName = "left_neutral";
    robot.getSwerveDrive().loadChoreoPath(pathName + ".0");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".1");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".2");

    return Commands.sequence(
        Commands.runOnce(
            () ->
                robot
                    .getSwerveDrive()
                    .getLocalization()
                    .resetPosition(mirrorPose(LEFT_START_POSE, rightSide))),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".0", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".1", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot.getSwerveDrive().followPath(pathName + ".2", rightSide),
        shootFuel.shoot());
  }

  private Command doubleNeutralCycle(boolean rightSide) {
    String pathName = "left_neutral";
    robot.getSwerveDrive().loadChoreoPath(pathName + ".0");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".1");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".2");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".3");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".4");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".5");

    return Commands.sequence(
        Commands.runOnce(
            () ->
                robot
                    .getSwerveDrive()
                    .getLocalization()
                    .resetPosition(mirrorPose(LEFT_START_POSE, rightSide))),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".0", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".1", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot.getSwerveDrive().followPath(pathName + ".2", rightSide),
        shootFuel.shootAllFuelStationary().withTimeout(20 - 13.2),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".3", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".4", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".5", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        shootFuel.shoot());
  }

  public Command bump(boolean rightSide) {
    String pathName = "left_neutral_bump";
    robot.getSwerveDrive().loadChoreoPath(pathName + ".0");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".1");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".2");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".3");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".4");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".5");

    return Commands.sequence(
            Commands.runOnce(
                () ->
                    robot
                        .getSwerveDrive()
                        .getLocalization()
                        .resetPosition(mirrorPose(LEFT_START_POSE, rightSide))),
            robot
                .getSwerveDrive()
                .followPath(pathName + ".0", rightSide)
                .deadlineFor(
                    robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
            robot
                .getSwerveDrive()
                .followPath(pathName + ".1", rightSide)
                .deadlineFor(
                    robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
            robot.getSwerveDrive().followPath(pathName + ".2", rightSide),
            shootFuel.shootAllFuelStationary().withTimeout(5),
            robot
                .getSwerveDrive()
                .followPath(pathName + ".3", rightSide)
                .deadlineFor(
                    robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
            robot
                .getSwerveDrive()
                .followPath(pathName + "4", rightSide)
                .deadlineFor(
                    robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
            robot.getSwerveDrive().followPath(pathName + ".5", rightSide),
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

  public Command greedyDoubleSweepBump(boolean rightSide) {
    String pathName = "greedy_double_sweep_bump";
    robot.getSwerveDrive().loadChoreoPath(pathName + ".0");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".1");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".2");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".3");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".4");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".5");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".6");

    return Commands.sequence(
        Commands.runOnce(
            () ->
                robot
                    .getSwerveDrive()
                    .getLocalization()
                    .resetPosition(mirrorPose(LEFT_START_POSE_ROTATED, rightSide))),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".0", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".1", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".2", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getHopper().unjam()),
        Commands.parallel(
            robot.getSwerveDrive().followPath(pathName + ".3", rightSide),
            shootFuel.shootAllFuelOnTheMove().withTimeout(6)),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".4", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".5", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getHopper().unjam()),
        Commands.parallel(
            robot.getSwerveDrive().followPath(pathName + ".6", rightSide),
            shootFuel.shootAllFuelOnTheMove().withTimeout(6)),
        robot.getSwerveDrive().followPath(pathName + ".7", rightSide));
  }

  public Command safeDoubleSweepBump(boolean rightSide) {
    String pathName = "safe_double_sweep_bump";
    robot.getSwerveDrive().loadChoreoPath(pathName + ".0");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".1");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".2");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".3");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".4");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".5");
    robot.getSwerveDrive().loadChoreoPath(pathName + ".6");

    return Commands.sequence(
        Commands.runOnce(
            () ->
                robot
                    .getSwerveDrive()
                    .getLocalization()
                    .resetPosition(mirrorPose(LEFT_START_POSE_ROTATED, rightSide))),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".0", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".1", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".2", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getHopper().unjam()),
        Commands.parallel(
            robot.getSwerveDrive().followPath(pathName + ".3", rightSide),
            shootFuel.shootAllFuelOnTheMove().withTimeout(6)),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".4", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake()),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".5", rightSide)
            .deadlineFor(robot.getIntakeExtension().extend(), robot.getHopper().unjam()),
        Commands.parallel(
            robot.getSwerveDrive().followPath(pathName + ".6", rightSide),
            shootFuel.shootAllFuelOnTheMove().withTimeout(6)),
        robot.getSwerveDrive().followPath(pathName + ".7", rightSide));
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

  public Command greedyLeftDoubleSweepBump() {
    return greedyDoubleSweepBump(false);
  }

  public Command greedyRightDoubleSweepBump() {
    return greedyDoubleSweepBump(true);
  }

  public Command safeLeftDoubleSweepBump() {
    return safeDoubleSweepBump(false);
  }

  public Command safeRightDoubleSweepBump() {
    return safeDoubleSweepBump(true);
  }

  public Command preload() {
    return shootFuel.shoot();
  }

  public Command center(boolean fromHub) {
    String pathName = fromHub ? "center_from_hub" : "center";

    return Commands.sequence(
        Commands.runOnce(
            () ->
                robot
                    .getSwerveDrive()
                    .getLocalization()
                    .resetPosition(
                        fromHub
                            ? new Pose2d(3.6155219078063965, 4.211485385894775, Rotation2d.k180deg)
                            : new Pose2d(
                                3.616589307785034, 5.050553798675537, Rotation2d.k180deg))),
        Commands.parallel(
            Commands.sequence(
                Commands.parallel(
                    robot.getSwerveDrive().followPath(pathName + ".0"),
                    robot
                        .getIntakeExtension()
                        .extend()), // Drive away from hub to where shooting can start
                robot
                    .getSwerveDrive()
                    .followPath(pathName + ".1")
                    .deadlineFor(
                        shootFuel.shootOnTheMove()), // Drive while shooting preload to depot setup
                // position
                shootFuel.shootAllFuelStationary().withTimeout(2) // Shoot any remaining fuel
                )),
        robot
            .getSwerveDrive()
            .followPath(pathName + ".2")
            .deadlineFor(
                robot.getIntakeExtension().extend(),
                robot.getIntakeRollers().intake()), // Intake from depot
        robot.getSwerveDrive().followPath(pathName + ".3"),
        shootFuel.shootAllFuelStationary(),
        robot.getSwerveDrive().followPath(pathName + ".4"));
  }

  private static Pose2d mirrorPose(Pose2d pose, boolean mirrored) {
    if (!mirrored) {
      return pose;
    }

    return new Pose2d(pose.getX(), 8.0692625 - pose.getY(), pose.getRotation().unaryMinus());
  }
}
