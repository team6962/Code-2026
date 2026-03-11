package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldPositions {
  public static final Translation2d START_POSITION = new Translation2d(0, 0); // dummy start pose
  public static final Translation2d ALLIANCE_ZONE_CENTER =
      new Translation2d(Inches.of(91.055).in(Meters), Inches.of(158.84).in(Meters));

  public static
  class Trench { // values need to be tuned and need to check if in alliance or neutral zone
    public static final Translation2d RIGHT_ALLIANCE =
        new Translation2d(
            Inches.of(115.61).in(Meters), // original = 165.61
            Inches.of(24.92).in(Meters));
    public static final Translation2d LEFT_ALLIANCE =
        new Translation2d(Inches.of(115.61).in(Meters), Inches.of(292.76).in(Meters));
    public static final Translation2d RIGHT_NEUTRAL =
        new Translation2d(Inches.of(248.61).in(Meters), Inches.of(24.92).in(Meters));
    public static final Translation2d LEFT_NEUTRAL =
        new Translation2d(
            Inches.of(248.61).in(Meters), // original pos = 198.61
            Inches.of(292.76).in(Meters));
    public static final Translation2d RIGHT_CENTER =
        new Translation2d(Inches.of(182.11).in(Meters), Inches.of(24.92).in(Meters));
    public static final Translation2d LEFT_CENTER =
        new Translation2d(Inches.of(182.11).in(Meters), Inches.of(292.76).in(Meters));
  }

  public static class Bump {
    public static final Translation2d RIGHT_ALLIANCE =
        new Translation2d(Inches.of(115.61).in(Meters), Inches.of(98.85).in(Meters));
    public static final Translation2d LEFT_ALLIANCE =
        new Translation2d(Inches.of(115.61).in(Meters), Inches.of(218.518).in(Meters));
    public static final Translation2d RIGHT_NEUTRAL =
        new Translation2d(Inches.of(248.61).in(Meters), Inches.of(98.85).in(Meters));
    public static final Translation2d LEFT_NEUTRAL =
        new Translation2d(Inches.of(248.61).in(Meters), Inches.of(218.518).in(Meters));
  }

  public static final Translation2d TOWER = new Translation2d(Inches.of(147.47), Inches.of(41.56));
  public static final Pose2d DEPOT_OUTSIDE = new Pose2d(1.518, 5.947, Rotation2d.kZero);
  public static final Pose2d DEPOT_INSIDE = new Pose2d(0.546, 5.947, Rotation2d.kZero);
  public static final Pose2d ENEMY_DEPOT_OUTSIDE = new Pose2d(15.410, 2.000, Rotation2d.kPi);
  public static final Pose2d ENEMY_DEPOT_INSIDE = new Pose2d(15.860, 2.000, Rotation2d.kPi);
  public static final Pose2d ENEMY_TRENCH = new Pose2d(12.000, 0.500, Rotation2d.kPi);
  public static final Pose2d OUTPOST = new Pose2d(0.6, 0.65, Rotation2d.fromDegrees(180));

  public static final Translation2d NEUTRAL_ZONE_CENTER =
      new Translation2d(Inches.of(325.61).in(Meters), Inches.of(158.84).in(Meters));
  public static final Translation2d HUB_FRONT =
      new Translation2d( // These are suspicious, should be double checked
          Inches.of(123.61).in(Meters), Inches.of(158.84).in(Meters));
  public static final Translation2d HUB_CENTER =
      new Translation2d( // These are suspicious, should be double checked
          Inches.of(182.11).in(Meters), Inches.of(158.84).in(Meters));
  public static final Translation2d HUB_BACK =
      new Translation2d(Inches.of(230.61).in(Meters), Inches.of(158.84).in(Meters));

  public static final Translation2d LEFT_END_OF_NEUTRAL_ZONE =
      new Translation2d(
          Inches.of(439.11).in(Meters), Inches.of(292.76).in(Meters)); // needs to be tuned
  public static final Translation2d RIGHT_END_OF_NEUTRAL_ZONE =
      new Translation2d(
          Inches.of(439.11).in(Meters), Inches.of(24.92).in(Meters)); // needs to be tuned
}
