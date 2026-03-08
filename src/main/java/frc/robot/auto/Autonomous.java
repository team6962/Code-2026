package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

/** Contains autonomous command sequences that can be selected on the dashboard. */
public class Autonomous {
  private RobotContainer robot;
  public final TrenchDriving trench;
  public final NeutralIntake neutralIntake;
  public final ShootFuel shootFuel;
  public final AutoDepot autoDepot;
  public final AutoOutpost autoOutpost;
  public final AutoEdgeIntake autoEdgeIntake;

  public Autonomous(RobotContainer robot) {
    this.robot = robot;
    this.trench = new TrenchDriving(robot);
    this.neutralIntake = new NeutralIntake(robot);
    this.shootFuel = new ShootFuel(robot);
    this.autoDepot = new AutoDepot(robot);
    this.autoOutpost = new AutoOutpost(robot, shootFuel);
    this.autoEdgeIntake = new AutoEdgeIntake(robot);
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

  public Command outpostThenNeutralCycle() {
    return Commands.sequence(
        trench.driveToNeutral(),
        neutralIntake.intake(Meters.of(1)),
        trench.driveToAlliance(),
        shootFuel.shootAllFuel(),
        autoOutpost.autoOutpost());
  }

  public Command leftEdgeCleanup() {
    return Commands.sequence(
        autoEdgeIntake.intakeToEdgeLeft(), trench.driveToAlliance(), shootFuel.shootAllFuel());
  }

  public Command RightEdgeCleanup() {
    return Commands.sequence(
        autoEdgeIntake.intakeToEdgeRight(), trench.driveToAlliance(), shootFuel.shootAllFuel());
  }
}
