package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterHoodConstants {
  public static final int MOTOR_CAN_ID = 20;
  public static final int CANDI_CAN_ID = 20;
  public static final String CANBUS = "subsystems";
  public static final DCMotor MOTOR_PHYSICS = DCMotor.getKrakenX44Foc(1);

  public static final MomentOfInertia MOMENT_OF_INERTIA = KilogramSquareMeters.of(0.04942);

  public static final Angle MIN_ANGLE = Degrees.of(17.95);
  public static final Angle MAX_ANGLE = Degrees.of(50);

  /** Distance between the pivot point and the center of mass of the hood. */
  public static final Distance ARM_LENGTH = Inches.of(6.85);

  /** Gravity compensation feedforward constant (volts). */
  public static final double kG = 0.22;

  public static final TalonFXConfiguration MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(176 / 3))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(30.0)
                  .withMotionMagicAcceleration(30.0)
                  .withMotionMagicJerk(0))
          .withSlot0(
              new Slot0Configs()
                  .withKP(80.0)
                  .withKD(1.0)
                  .withKV(12.0 / (7368.0 / 60.0) * 176.0 / 3.0)
                  .withKS(0.0)
                  .withKA(0.03)
                  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
              // Don't add kG here, instead use ShooterHoodConstants.kG
              )
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(true)
                  .withReverseSoftLimitEnable(true)
                  .withForwardSoftLimitThreshold(MAX_ANGLE)
                  .withReverseSoftLimitThreshold(MIN_ANGLE))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake));
  public static final CANdiConfiguration CANDI_CONFIGURATION = new CANdiConfiguration();
}
