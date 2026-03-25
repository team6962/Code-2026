package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.shoot.AutoShoot;

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
  public final AutoShoot autoPassLeft;
  public final AutoShoot autoPassRight;

  public Autonomous(RobotContainer robot) {
    this.robot = robot;
    this.trench = new TrenchDriving(robot);
    this.neutralIntake = new NeutralIntake(robot);
    this.shootFuel = new ShootFuel(robot);
    this.autoDepot = new AutoDepot(robot);
    this.autoOutpost = new AutoOutpost(robot, shootFuel);
    this.autoEdgeIntake = new AutoEdgeIntake(robot);
    this.collectFuelFromHub = new CollectFuelFromHub(robot);
    this.autoPassLeft = new AutoShoot(robot, () -> AutoShoot.PASS_LEFT_TRANSLATION);
    this.autoPassRight = new AutoShoot(robot, () -> AutoShoot.PASS_RIGHT_TRANSLATION);
  }

  private static Pose2d LEFT_START_POSE =
      new Pose2d(4.436294078826904, 7.646793365478516, new Rotation2d());

  private Command singleNeutralCycle(boolean rightSide) {
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
        robot.getSwerveDrive().followPath("left_neutral.1", rightSide),
        shootFuel.shoot());
  }

  private Command doubleNeutralCycle(boolean rightSide) {
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
        robot.getSwerveDrive().followPath("left_neutral.1", rightSide),
        shootFuel.shootAllFuelStationary(),
        robot
            .getSwerveDrive()
            .followPath("left_neutral.2", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
        shootFuel.shoot());
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
    return shootFuel
        .shoot()
        .deadlineFor(robot.getIntakeExtension().extend(), robot.getIntakeRollers().intake());
  }

  public Command passCycle(boolean rightSide) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                robot
                    .getSwerveDrive()
                    .getLocalization()
                    .resetPosition((mirrorPose(LEFT_START_POSE, rightSide)))),
        // shootFuel.shoot(),
        robot.getSwerveDrive().followPath("pass_cycle.0", rightSide),
        robot
            .getSwerveDrive()
            .followPath("pass_cycle.1", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(),
                robot.getIntakeRollers().intakeFast(),
                rightSide ? autoPassLeft : autoPassRight,
                robot.getHopper().feed()),
        robot
            .getSwerveDrive()
            .followPath("pass_cycle.2", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(), robot.getIntakeRollers().intakeFast()),
        robot
            .getSwerveDrive()
            .followPath("pass_cycle.3", rightSide)
            .deadlineFor(
                robot.getIntakeExtension().extend(),
                robot.getIntakeRollers().intake(),
                shootFuel.shoot()),
        shootFuel.shoot());
  }

  public Command leftPassCycle() {
    return passCycle(false);
  }

  public Command rightPassCycle() {
    return passCycle(true);
  }

  private static Pose2d mirrorPose(Pose2d pose, boolean mirrored) {
    if (!mirrored) {
      return pose;
    }

    return new Pose2d(pose.getX(), 8.0692625 - pose.getY(), pose.getRotation().unaryMinus());
  }
}
