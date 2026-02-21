package frc.robot.subsystems.visualizer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Constants used for visualizing the robot in AdvantageScope. */
public class RobotVisualizationConstants {
  /** The translation of the shooter component relative to the robot's origin. */
  public static Translation3d shooterTranslation =
      new Translation3d(-0.17145, 0.17145, 0.298665265);

  /** The translation of the hood component relative to the shooter. */
  public static Translation3d hoodTranslation = new Translation3d(0.1143700024, 0, 0.13335);

  /** The angle of the intake relative to the horizontal plane. */
  public static Angle intakeAngle = Degrees.of(-18);

  // Hopper dimensions for visualization and simulation
  public static int hopperExtendedLength = 4;
  public static int hopperRetractedLength = 2;
  public static int hopperWidth = 4;
  public static int maxExtendedFuel = 48;
  public static int maxRetractedFuel = 24;
  public static Distance fuelSpacing = Inches.of(6);
}
