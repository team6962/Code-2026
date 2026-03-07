package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
    // drive to trench on neutral side
    public static final Translation2d RIGHT_NEUTRAL =
        new Translation2d(Inches.of(248.61).in(Meters), Inches.of(24.92).in(Meters));
    public static final Translation2d LEFT_NEUTRAL =
        new Translation2d(
            Inches.of(248.61).in(Meters), // original pos = 198.61
            Inches.of(292.76).in(Meters));

    }
    
    public static final Translation2d NEUTRAL_ZONE_CENTER = new Translation2d(
        Inches.of(325.61).in(Meters),
        Inches.of(158.84).in(Meters));
    public static final Translation2d HUB_FRONT = new Translation2d( // These are suspicious, should be double checked
        Inches.of(158.84).in(Meters),
        Inches.of(123.61).in(Meters));
    public static final Translation2d HUB_CENTER = new Translation2d( // These are suspicious, should be double checked
        Inches.of(182.11).in(Meters),
        Inches.of(158.84).in(Meters));
}
