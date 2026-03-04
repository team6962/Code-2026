package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoSegments {
    private RobotContainer robot;

    private final Pose2d START_POSE = new Pose2d(0, 0, Rotation2d.kZero); // dummy start pose

    public AutoSegments(RobotContainer robot) {
        this.robot = robot;
    }

    // Example auto segment - Not a full autonomous routine, but one piece of it
    public Command driveToStart() {
        return robot.getSwerveDrive().driveTo(START_POSE);
    }

    public Command driveToMiddleAlliance(){
        return robot.getSwerveDrive().driveTo(new Translation2d(Inches.of(90.78).in(Meters), Inches.of(158.32).in(Meters)));
        //Vertical 158.32 Horizontal 90.78
    }
}
