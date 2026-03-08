package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

/**
 * Contains autonomous command sequences that can be selected on the dashboard.
 */
public class Autonomous {
  private RobotContainer robot;
  private TrenchDriving trench;

  public Autonomous(RobotContainer robot) {
    this.robot = robot;
    this.trench = new TrenchDriving(robot);
  }

  public Command neutralCycle() {
    return Commands.sequence(trench.driveToNeutral(), trench.driveToAlliance());
  }
}
