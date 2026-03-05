package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldPositions {
    public static final Translation2d START_POSITION = new Translation2d(0, 0); // dummy start pose
    public static final Translation2d ALLIANCE_ZONE_CENTER = new Translation2d(
        Inches.of(91.055).in(Meters),
        Inches.of(158.84).in(Meters));

    public static class Trench {
        public static final Translation2d RIGHT_FRONT = new Translation2d(
            Inches.of(165.61).in(Meters),
            Inches.of(24.92).in(Meters));
        public static final Translation2d LEFT_FRONT = new Translation2d(
            Inches.of(165.61).in(Meters),
            Inches.of(292.76).in(Meters));
        public static final Translation2d RIGHT_BACK = new Translation2d(
            Inches.of(198.61).in(Meters),
            Inches.of(24.92).in(Meters));
        public static final Translation2d LEFT_BACK = new Translation2d(
            Inches.of(198.61).in(Meters),
            Inches.of(292.76).in(Meters));
    }
    
    public static final Translation2d NEUTRAL_ZONE_CENTER = new Translation2d(
        Inches.of(325.61).in(Meters),
        Inches.of(158.84).in(Meters));
    public static final Translation2d HUB_CENTER = new Translation2d(
        Inches.of(158.84).in(Meters),
        Inches.of(182.11).in(Meters));
}