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

    public AutoSegments(RobotContainer robot) {
        this.robot = robot;
    }
    
    public Command driveToStart() {
        return robot.getSwerveDrive().driveTo(new Pose2d(FieldPositions.START_POSITION, Rotation2d.kZero));
    }

    public Command driveToMiddleAlliance(){
        return robot.getSwerveDrive().driveTo(FieldPositions.ALLIANCE_ZONE_CENTER);
    }

    public Command driveToMiddleNeutral(){
        return robot.getSwerveDrive().driveTo(FieldPositions.NEUTRAL_ZONE_CENTER);
    }

    /*
     * drive to left trench from alliance zone
     */
    public Command driveToLeftTrenchAlliance(){
        return robot.getSwerveDrive().driveTo(FieldPositions.Trench.LEFT_ALLIANCE);
    }

    /*
     * drive to left trench from neutral zone
     */
    public Command driveToLeftTrenchNeutral(){
        return robot.getSwerveDrive().driveTo(FieldPositions.Trench.LEFT_NEUTRAL);
    }

    /*
     * drive to right trench from alliance zone
     */
    public Command driveToRightTrenchAlliance(){
        return robot.getSwerveDrive().driveTo(FieldPositions.Trench.RIGHT_ALLIANCE);
    }

    /*
     * drive to right trench from neutral zone
     */
    public Command driveToRightTrenchNeutral(){
        return robot.getSwerveDrive().driveTo(FieldPositions.Trench.RIGHT_NEUTRAL);
    }

    public Command driveToHub(){
        return robot.getSwerveDrive().driveTo(FieldPositions.HUB_FRONT);
    }
}
