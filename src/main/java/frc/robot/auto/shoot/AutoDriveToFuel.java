package frc.robot.auto.shoot;

import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class AutoDriveToFuel {
    private RobotContainer robot;

    public AutoDriveToFuel(RobotContainer robot){
        this.robot = robot;
    }
    public Command AutoDriveToFuel() {
        return Commands.defer(
            ()-> {
                Translation2d fuelPosition = robot.getFuelLocalization().getClumpPosition();
                if (fuelPosition == null) {
                    return Commands.none();
                }
                    
                return
                    Commands.parallel(robot.getIntakeRollers().intake(), robot.getHopper().load(),                     
                        robot.getSwerveDrive()
                        .driveVelocity(
                            ()->
                        ChassisSpeeds.fromRobotRelativeSpeeds(
                        new ChassisSpeeds(1, 0, 0), 
                            new Rotation2d(robot.getSwerveDrive().getYaw()))));

        }, Set.of(robot.getIntakeExtension(),
                robot.getIntakeRollers(),
                robot.getSwerveDrive().useTranslation(),
                robot.getSwerveDrive().useRotation()));
    }
}