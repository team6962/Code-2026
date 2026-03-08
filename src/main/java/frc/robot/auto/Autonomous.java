package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.logging.LoggingUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.TrenchDriving.Trench;
import java.util.Set;

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

  public Command neutralCycle() {
    return Commands.sequence(
        trench.driveToNeutral(),
        neutralIntake.intake(Meters.of(1)),
        trench.driveToAlliance(),
        shootFuel.shootAllFuel(),
        trench.driveToNeutral(),
        neutralIntake.intake(Meters.of(1), Meters.of(2.5)),
        trench.driveToAlliance(),
        shootFuel.shootAllFuel());
  }

  public Command depotThenNeutralCycle() {
    return Commands.sequence(
        autoDepot.autoDepot(),
        trench.driveToNeutral().deadlineFor(shootFuel.shootAllFuel()),
        neutralIntake.intake(Meters.of(1)),
        trench.driveToAlliance(),
        shootFuel.shootAllFuel());
  }

  public Command neutralCycleThenOutpost() {
    return Commands.defer(
        () -> {
          Trench nearestTrench = trench.getNearestTrench();

          return LoggingUtil.logCommand(
              "neutralCycleThenOutpost",
              Commands.sequence(
                  LoggingUtil.logCommand("driveToNeutral", trench.driveToNeutral()),
                  LoggingUtil.logCommand("neutralIntake", neutralIntake.intake(Meters.of(1))),
                  LoggingUtil.logCommand(
                      "driveToAlliance",
                      trench.driveToAlliance(nearestTrench, Rotation2d.fromDegrees(180))),
                  LoggingUtil.logCommand("shootFuel", shootFuel.shootAllFuel()),
                  LoggingUtil.logCommand("autoOutpost", autoOutpost.autoOutpost())));
        },
        Set.of(
            robot.getSwerveDrive().useRotation(),
            robot.getSwerveDrive().useTranslation(),
            robot.getIntakeRollers(),
            robot.getIntakeExtension(),
            robot.getHopper().getBeltFloor(),
            robot.getHopper().getKicker()));
  }

  public Command leftEdgeCleanup() {
    return Commands.sequence(
        autoEdgeIntake.intakeToEdgeLeft(), trench.driveToAlliance(), shootFuel.shootAllFuel());
  }

  public Command rightEdgeCleanup() {
    return Commands.sequence(
        autoEdgeIntake.intakeToEdgeRight(), trench.driveToAlliance(), shootFuel.shootAllFuel());
  }

  public Command intakeBehindHubLeft() {
    return Commands.sequence(
        trench.driveToNeutral(Trench.LEFT),
        collectFuelFromHub.intakeBehindHubLeft(),
        trench.driveToAlliance(Trench.LEFT),
        shootFuel.shootAllFuel());
  }

  public Command intakeBehindHubRight() {
    return Commands.sequence(
        trench.driveToNeutral(Trench.RIGHT),
        collectFuelFromHub.intakeBehindHubRight(),
        trench.driveToAlliance(Trench.RIGHT),
        shootFuel.shootAllFuel());
  }
}
