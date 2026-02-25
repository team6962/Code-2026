package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveFixedShooter {
    private RobotContainer robot;
    private final Translation2d LEFT_HUB_POSITION = new Translation2d(
        Inches.of(182.11).in(Meters), 
        Inches.of(158.84).in(Meters)
    );
    private final Translation2d RIGHT_HUB_POSITION = new Translation2d(
        Inches.of(469.11).in(Meters), 
        Inches.of(158.84).in(Meters)
    );

    public DriveFixedShooter(RobotContainer robot){
        this.robot = robot;
    }
    public Command driveToLeftPosition(double radius){
        Translation2d shotPosition = LEFT_HUB_POSITION.plus( new Translation2d(-radius/Math.sqrt(2.00),radius/Math.sqrt(2.00)));
        return robot.getSwerveDrive().driveTo(new Pose2d(shotPosition,new Rotation2d(Degrees.of(-45.0))));
    }
    public Command driveToRightPosition(double radius){
        Translation2d shotPosition = LEFT_HUB_POSITION.plus( new Translation2d(-radius/Math.sqrt(2.00),-radius/Math.sqrt(2.00)));
        return robot.getSwerveDrive().driveTo(new Pose2d(shotPosition,new Rotation2d(Degrees.of(45.0))));
    }
}
//hub cord inches (left) (182.11,158.84) 
//hub cord inches (right) (469.11,158.84) 