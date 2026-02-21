package frc.robot.auto;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.ShooterHoodConstants;

public class AutoPassing {
    private RobotContainer robot;

    private final Translation2d LEFT_PASS_TARGET = new Translation2d(1.525, 7.022);
    private final Translation2d RIGHT_PASS_TARGET = new Translation2d(6.67, 7.022); // x is not correct yet

    public AutoPassing(RobotContainer robot) {
        this.robot = robot;
    }

    public Command passLeft() {
        return passTo(LEFT_PASS_TARGET);
    }

    public Command passRight() {
        return passTo(RIGHT_PASS_TARGET);
    }

    public Command passTo(Translation2d target) {
        return Commands.sequence(
            // Move turret first
            robot.getTurret().moveTo(
                () -> {
                    // Get translation to the left side of the field
                    return robot.getSwerveDrive().getPosition2d().getTranslation()
                        .minus(target).getAngle().getMeasure() // Convert the Translation2d into an angle
                        .minus(robot.getSwerveDrive().getHeading()); // Subtract the robot's current heading to get the angle that the turret should rotate to
                }
            ),

            // Retract shooter hood
            robot.getShooterHood().moveTo(ShooterHoodConstants.MAX_ANGLE),

            // Activate the shooter rollers
            robot.getShooterRollers().shoot(RotationsPerSecond.of(1)) // arbitrary shooting speed for now
        );
    }
}
