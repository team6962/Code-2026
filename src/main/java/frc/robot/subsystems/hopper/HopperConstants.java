package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class HopperConstants {
    //kicker
    public static final int KICKER_DEVICE_ID = 31;
    public static final double KICKER_MOMENT_OF_INERTIA = 0.005;
    public static final double KICKER_GEAR_RATIO = 2.5;
    public static final TalonFXConfiguration KICKER_MOTOR_CONFIGURATION = 
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(KICKER_GEAR_RATIO))
          .withSlot0(new Slot0Configs().withKP(0.02)) // needs to be tuned
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(120)) //everything needs to be tuned
                  .withSupplyCurrentLimit(Amps.of(60)) // current limits might not be neccesary
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true));


    //belt floor
    public static final int BELT_FLOOR_MOTOR_CAN_ID = 30;
    public static final String BELT_FLOOR_CANBUS_NAME = "subsystem";
    public static final TalonFXConfiguration BELT_FLOOR_MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(4 / 3))
          .withSlot0(new Slot0Configs().withKV(0.124137931).withKP(0.01))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(120)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(80));
    
}