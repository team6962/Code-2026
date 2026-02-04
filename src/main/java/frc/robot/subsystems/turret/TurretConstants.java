package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

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

  // Hall Effect Sensor Configuration
  public static final int HALL_SENSOR_DIO_CHANNEL = 0; // Change this to match your CANdi DIO channel
  public static final double ZERO_POSITION_ANGLE = 0.0; // The angle to set when hall sensor is triggered (in rotations)
  public static final double ZEROING_SPEED = 0.1; // Speed at which to move during zeroing (rotations/sec)

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
  public static final String TUNABLE_CRUISE_VELOCITY_KEY = "Turret Rotation/Motion Magic/Cruise Velocity";
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