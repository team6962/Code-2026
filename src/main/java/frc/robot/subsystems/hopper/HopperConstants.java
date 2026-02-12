package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;

public class HopperConstants {
  // kicker
  public static final int KICKER_DEVICE_ID = 31;
  public static final double KICKER_MOMENT_OF_INERTIA = 0.005;

  // belt floor
  public static final int BELT_FLOOR_MOTOR_CAN_ID = 30;
  public static final String BELT_FLOOR_CANBUS_NAME = "subsystem";
  public static final DCMotor BELT_FLOOR_MOTOR_PHYSICS = DCMotor.getKrakenX60Foc(1);
  public static final double BELT_FLOOR_GEARING = 2.5;
  public static final MomentOfInertia BELT_FLOOR_MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.02);
  public static final double BELT_FLOOR_PULLEY_RADIUS = 0.51;
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
}
