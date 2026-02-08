package frc.robot.subsystems.shooterrollers;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;

/*
 * sets constants for the shooter roller
 */
public final class ShooterRollersConstants {
  public static final int MOTOR_CAN_ID_1 = 21;
  public static final int MOTOR_CAN_ID_2 = 22;
  public static final DCMotor MOTOR_PHYSICS = DCMotor.getKrakenX60Foc(2);
  public static final String CANBUS_NAME = "subsystems";
  public static final MomentOfInertia MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.003072714);
  public static final TalonFXConfiguration MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs())
          .withSlot0(new Slot0Configs().withKV(0.124137931).withKS(0.423).withKP(0.1))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(120)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(80))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
}
