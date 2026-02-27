package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public class HopperConstants {
  public static final String CANBUS_NAME = "subsystems";
  // kicker
  public static final int KICKER_DEVICE_ID = 31;
  public static final double KICKER_MOMENT_OF_INERTIA = 0.005;
  public static final double KICKER_GEAR_RATIO = 2.5;
  public static final TalonFXConfiguration KICKER_MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(KICKER_GEAR_RATIO))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(120)) // everything needs to be tuned
                  .withSupplyCurrentLimit(Amps.of(60)) // current limits might not be neccesary
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true));

  // belt floor
  public static final int BELT_FLOOR_MOTOR_CAN_ID = 30;
  public static final DCMotor BELT_FLOOR_MOTOR_PHYSICS = DCMotor.getKrakenX60Foc(1);
  public static final double BELT_FLOOR_GEARING = 2.5;
  public static final MomentOfInertia BELT_FLOOR_MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.02);
  public static final Distance BELT_FLOOR_PULLEY_RADIUS = Inches.of(0.51);
  public static final TalonFXConfiguration BELT_FLOOR_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(BELT_FLOOR_GEARING))
          .withSlot0(new Slot0Configs().withKV(0.124137931).withKP(0.01)) // temp change later
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(80)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(40));
  // Sensors
  // The CAN IDs for the sensors: kicker, upper hopper, and lower hopper.
  public static final int KICKER_SENSOR_CAN_ID = 30;
  public static final int UPPER_HOPPER_CAN_ID = 32;
  public static final int LOWER_HOPPER_CAN_ID = 31;

  // Threshold distances for determining if the Kicker is full.
  public static final Distance KICKER_SENSOR_FULL_THRESHOLD = Inches.of(5.0);

  // Threshold distances for determining if the Kicker is empty
  public static final Distance KICKER_SENSOR_EMPTY_THRESHOLD = Inches.of(16.0);

  // Threshold distances for determining if the Hopper is full
  public static final Distance UPPER_HOPPER_SENSOR_FULL_THRESHOLD =
      Inches.of(15.0); // This isn't the correct distance

  // Threshold distances for determining if the Hopper is empty
  public static final Distance LOWER_HOPPER_SENSOR_EMPTY_THRESHOLD = Inches.of(23.0);

  public static final CANrangeConfiguration KICKER_SENSOR_CONFIGURATION =
      new CANrangeConfiguration()
          .withFovParams(
              new FovParamsConfigs()
                  .withFOVRangeX(Degrees.of(6.75))
                  .withFOVRangeY(Degrees.of(6.75)));

  public static final CANrangeConfiguration UPPER_HOPPER_CONFIGURATION =
      new CANrangeConfiguration()
          .withFovParams(
              new FovParamsConfigs().withFOVRangeX(Degrees.of(27)).withFOVRangeY(Degrees.of(6.75)));
  public static final CANrangeConfiguration LOWER_HOPPER_CONFIGURATION =
      new CANrangeConfiguration()
          .withFovParams(
              new FovParamsConfigs()
                  .withFOVRangeX(Degrees.of(6.75))
                  .withFOVRangeY(Degrees.of(6.75)));
}
