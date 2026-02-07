package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

/** Constants for the Turret subsystem */
public class TurretConstants {
  // CAN IDs and bus name
  public static final int MOTOR_CAN_ID = 24;
  public static final int HALL_SENSOR_CANDI_CAN_ID = 20;
  public static final String CAN_BUS_NAME = "subsystems";

  // PID constants
  public static final double kP = 1.5;
  public static final double kD = 0.025;
  public static final double kS = 0.155;
  public static final double kV = 3.371;
  public static final double kA = 0.0;

  // Motion Magic constants
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 1.0;
  public static final double MOTION_MAGIC_ACCELERATION = 1.0;

  // Current limits
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);

  /** The gear ratio between the motor's shaft and turret mechanism. */
  public static final double SENSOR_TO_MECHANISM_RATIO = 34.5;

  /** The direction that is considered positive rotation for the motor. */
  public static final InvertedValue MOTOR_INVERSION = InvertedValue.Clockwise_Positive;

  /** The neutral mode of the motor (e.g., brake or coast). This will only
   * be used once the turret has been zeroed, as the turret is set to coast mode
   * until it has been zeroed to allow it to be manually moved.
   */
  public static final NeutralModeValue MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;

  /**
   * The minimum angle that the turret can be at to trigger the hall sensor. This is used when the
   * turret passes through the sensor moving from higher to lower angles. Set to null to disable
   * zeroing when moving in this direction.
   */
  public static final Angle MINIMUM_HALL_SENSOR_TRIGGER_ANGLE = Degrees.of(0.0);

  /**
   * The maximum angle that the turret can be at to trigger the hall sensor. This is used when the
   * turret passes through the sensor moving from lower to higher angles. Set to null to disable
   * zeroing when moving in this direction.
   */
  public static final Angle MAXIMUM_HALL_SENSOR_TRIGGER_ANGLE = Degrees.of(5.0);

  /**
   * The moment of inertia of the turret. This is used only for simulation and is not used in
   * the actual robot code. This value is calculated from the turret CAD model and is in kg*m^2.
   */
  public static final double MOMENT_OF_INERTIA = 0.09803;

  /**
   * The minimum angle that the turret can be at. This is used to prevent the turret from
   * trying to move beyond its physical limits.
   */
  public static final Angle MIN_ANGLE = Degrees.of(0);

  /**
   * The maximum angle that the turret can be at. This is used to prevent the turret from trying to
   * move beyond its physical limits.
   */
  public static final Angle MAX_ANGLE = Degrees.of(400);
}
