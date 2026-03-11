package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoEnemyDepot {
    private RobotContainer robot;

    public AutoEnemyDepot(RobotContainer robot) {
        this.robot = robot;
    }

    /**
     * Function to drive to intake fuel from enemy depot
     *
     * @return Command that drives robot to enemy Depot and intakes fuel.
     */
    public Command autoEnemyDepot() {
        return Commands.sequence(
            robot.getSwerveDrive().driveTo(FieldPositions.ENEMY_DEPOT_OUTSIDE),
            robot.getIntakeExtension().extend(),
            Commands.parallel(
                robot.getSwerveDrive().driveTo(FieldPositions.ENEMY_DEPOT_INSIDE),
                robot.getIntakeRollers().intake().withTimeout(2)));
    }
}
