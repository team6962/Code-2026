package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoDepot {
  private RobotContainer robot;

  public AutoDepot(RobotContainer robot) {
    this.robot = robot;
  }

  /**
   * Function to drive to intake fuel from depot
   *
   * @return Command that drives robot to Depot and intakes fuel.
   */
  public Command autoDepot() {
    return Commands.sequence(
        robot.getSwerveDrive().driveTo(FieldPositions.DEPOT_OUTSIDE),
        robot.getIntakeExtension().extend(),
        Commands.parallel(
            robot.getSwerveDrive().driveTo(FieldPositions.DEPOT_INSIDE),
            robot.getIntakeRollers().intake()));
  }
}
