package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldPositions {
    public static final Translation2d START_POSITION = new Translation2d(0, 0); // dummy start pose
    public static final Translation2d ALLIANCE_ZONE_CENTER = new Translation2d(
        Inches.of(90.78).in(Meters),
        Inches.of(158.32).in(Meters));
}
