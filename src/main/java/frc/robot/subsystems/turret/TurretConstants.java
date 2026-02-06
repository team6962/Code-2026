package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

/** Constants for the Turret subsystem */
public class TurretConstants {
  // Motor Configuration
  public static final int MOTOR_CAN_ID = 24;
  public static final String CAN_BUS_NAME = "subsystems";

  // PID Consts
  public static final double kP = 0.7;
  public static final double kD = 0.025;
  public static final double kS = 0.0;
  public static final double kV = 4.2827;
  public static final double kA = 0.0;

  // Motion Magic Consts
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 1.0;
  public static final double MOTION_MAGIC_ACCELERATION = 1.0;

  // Mechanical Consts
  public static final double SENSOR_TO_MECHANISM_RATIO = 34.5;

  // Inverted value
  public static final InvertedValue MOTOR_INVERSION = InvertedValue.Clockwise_Positive;

  // Neutral mode
  public static final NeutralModeValue MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;

  // Current limits
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);

  // Hall Effect Sensor Configuration
  public static final int HALL_SENSOR_CANDI = 20;

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

  // Simulation Consts
  public static final double MOMENT_OF_INERTIA = 0.09803;
  public static final int NUM_MOTORS = 1;
  public static final double SIMULATION_UPDATE_PERIOD = 0.02;

  // Tunable Key DogLog Paths
  public static final String TUNABLE_ANGLE_KEY = "Turret Rotation/Input Angle";
  public static final String TUNABLE_KP_KEY = "Turret Rotation/PID/kP";
  public static final String TUNABLE_KD_KEY = "Turret Rotation/PID/kD";
  public static final String TUNABLE_KS_KEY = "Turret Rotation/Feedforward/kS";
  public static final String TUNABLE_KV_KEY = "Turret Rotation/Feedforward/kV";
  public static final String TUNABLE_KA_KEY = "Turret Rotation/Feedforward/kA";
  public static final String TUNABLE_CRUISE_VELOCITY_KEY =
      "Turret Rotation/Motion Magic/Cruise Velocity";
  public static final String TUNABLE_ACCELERATION_KEY = "Turret Rotation/Motion Magic/Acceleration";

  // Logging Keys
  public static final String LOG_ANGULAR_VELOCITY = "Turret Rotation/Angular Velocity";
  public static final String LOG_MOTOR_VOLTAGE = "Turret Rotation/Motor Voltage";
  public static final String LOG_MOTOR_POSITION = "Turret Rotation/Motor Position angle";
  public static final String LOG_ANGULAR_ACCELERATION = "Turret Rotation/Angular Acceleration";
  public static final String LOG_SUPPLY_CURRENT = "Turret Rotation/Angular Supply Current";
  public static final String LOG_CONTROL_REQUEST = "Turret Rotation/Control Request";
  public static final String LOG_HALL_SENSOR_TRIGGERED = "Turret Rotation/Hall Sensor Triggered";
  public static final String LOG_IS_ZEROED = "Turret Rotation/Is Zeroed";

  // Default Tunable Values
  public static final double DEFAULT_ANGLE_INPUT = 0.0;

  // limit angles
  public static final Angle MIN_ANGLE = Degrees.of(0);
  public static final Angle MAX_ANGLE = Degrees.of(400);
}
