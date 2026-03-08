package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.ShootFuel;


public class AutoOutpost {
    private final RobotContainer robot;
    private final ShootFuel shootFuel;

    public AutoOutpost(RobotContainer robot, ShootFuel shootFuel) {
        this.robot = robot;
        this.shootFuel = shootFuel;
    }

    public Command autoOutpost() { 
        return Commands.sequence(
            robot.getSwerveDrive().driveTo(FieldPositions.OUTPOST),
            shootFuel.shootAllFuel().withTimeout(2.0)
        );
    }
}