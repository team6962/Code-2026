package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Constants for the Turret subsystem */
public class TurretConstants {
  // CAN IDs and bus name
  public static final int MOTOR_CAN_ID = 24;
  public static final int HALL_SENSOR_CANDI_CAN_ID = 20;
  public static final String CAN_BUS_NAME = "subsystems";

  // PID constants
  public static final double kP = 150.0;
  public static final double kD = 3.0;
  public static final double kS = 0.315;
  public static final double kV = 4.283;
  public static final double kA = 0.2;

  // Motion Magic constants
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 10.0;
  public static final double MOTION_MAGIC_ACCELERATION = 10.0;

  // Current limits
  public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);

  /** The gear ratio between the motor's shaft and turret mechanism. */
  public static final double SENSOR_TO_MECHANISM_RATIO = 34.5;

  /** The direction that is considered positive rotation for the motor. */
  public static final InvertedValue MOTOR_INVERSION = InvertedValue.CounterClockwise_Positive;

  /**
   * The neutral mode of the motor (e.g., brake or coast). This will only be used once the turret
   * has been zeroed, as the turret is set to coast mode until it has been zeroed to allow it to be
   * manually moved.
   */
  public static final NeutralModeValue MOTOR_NEUTRAL_MODE = NeutralModeValue.Brake;

  /**
   * The minimum angle that the turret can be at to trigger the hall sensor. This is used when the
   * turret passes through the sensor moving from higher to lower angles. Set to null to disable
   * zeroing when moving in this direction.
   */
  public static final Angle MINIMUM_HALL_SENSOR_TRIGGER_ANGLE = Radians.of(-0.720971);

  /**
   * The maximum angle that the turret can be at to trigger the hall sensor. This is used when the
   * turret passes through the sensor moving from lower to higher angles. Set to null to disable
   * zeroing when moving in this direction.
   */
  public static final Angle MAXIMUM_HALL_SENSOR_TRIGGER_ANGLE = Radians.of(-0.664565);

  /**
   * The minimum angle that the turret can be at. This is used to prevent the turret from trying to
   * move beyond its physical limits.
   */
  public static final Angle MIN_ANGLE = Degrees.of(-45);

  /**
   * The maximum angle that the turret can be at. This is used to prevent the turret from trying to
   * move beyond its physical limits.
   */
  public static final Angle MAX_ANGLE = Degrees.of(405);

  /**
   * The voltage to apply to the turret motor when the operator is manually controlling the turret
   * with fine control (e.g., holding a button to slowly rotate the turret). This should be low
   * enough to allow for precise control, but high enough to overcome static friction and move the
   * turret at a reasonable speed.
   */
  public static final Voltage FINE_CONTROL_VOLTAGE = Volts.of(0.5);

  // Simulation
  public static final DCMotor SIMULATED_MOTOR = DCMotor.getKrakenX44Foc(1);
  public static final double MOMENT_OF_INERTIA = 0.09803; // kg*m^2, calculated in CAD

  public static final double simulationKV =
      SIMULATED_MOTOR.nominalVoltageVolts
          / Units.radiansToRotations(SIMULATED_MOTOR.freeSpeedRadPerSec)
          * SENSOR_TO_MECHANISM_RATIO;
  public static final double simulationKA =
      MOMENT_OF_INERTIA
          * SIMULATED_MOTOR.nominalVoltageVolts
          / SIMULATED_MOTOR.stallTorqueNewtonMeters
          / SENSOR_TO_MECHANISM_RATIO
          * 2
          * Math.PI;
}
