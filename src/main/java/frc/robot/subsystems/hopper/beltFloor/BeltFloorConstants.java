package frc.robot.subsystems.hopper.beltFloor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class BeltFloorConstants {
    public static final int BELT_FLOOR_MOTOR_CAN_ID = 30;
    public static final String CANBUS_NAME = "subsystem";
    public static final TalonFXConfiguration MOTOR_CONFIGURATION =
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


