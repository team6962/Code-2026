package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

/** Contains autonomous command sequences that can be selected on the dashboard. */
public class Autonomous {
  private RobotContainer robot;
  private TrenchDriving trench;
  private NeutralIntake neutralIntake;
  private ShootFuel shootFuel;

  public Autonomous(RobotContainer robot) {
    this.robot = robot;
    this.trench = new TrenchDriving(robot);
    this.neutralIntake = new NeutralIntake(robot);
    this.shootFuel = new ShootFuel(robot);
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
        shootFuel.shootAllFuel()
    );
  }
}
