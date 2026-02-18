package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.units.measure.Distance;

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

  // Sensors
  // The CAN IDs for the sensors: kicker, upper hopper, and lower hopper.
  public static final int KICKER_SENSOR_CAN_ID = 30;
  public static final int UPPER_HOPPER_CAN_ID = 32;
  public static final int LOWER_HOPPER_CAN_ID = 31;

  // Threshold distances for determining if the Kicker is full.
  public static final Distance KICKER_SENSOR_FULL_THRESHOLD =
      Inches.of(1.0); // This isn't the correct distance

  // Threshold distances for determining if the Kicker is empty
  public static final Distance KICKER_SENSOR_EMPTY_THRESHOLD =
      Inches.of(2.0); // This isn't the correct distance

  // Threshold distances for determining if the Hopper is full
  public static final Distance UPPER_HOPPER_SENSOR_FULL_THRESHOLD =
      Inches.of(1.0); // This isn't the correct distance

  // Threshold distances for determining if the Hopper is empty
  public static final Distance LOWER_HOPPER_SENSOR_EMPTY_THRESHOLD =
      Inches.of(1.0); // This isn't the correct distance

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
