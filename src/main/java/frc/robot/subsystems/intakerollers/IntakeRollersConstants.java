package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeRollersConstants {
  public static final double GEAR_RATIO = 32 / 18;
  public static final double MOMENT_OF_INERTIA = 0.00074271944;
  public static final int DEVICE_ID = 41;

  public static final TalonFXConfiguration MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
          // fake numbers end here
          .withSlot0(new Slot0Configs().withKP(0.02)) // needs to be tuned
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(120))
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true));
}
