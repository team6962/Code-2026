package frc.robot.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class IntakeExtensionConstants {

  public static final int MOTOR_CAN_ID = 40;
  public static final DCMotor MOTOR_PHYSICS = DCMotor.getKrakenX60Foc(1);
  public static final Mass MOVING_MASS = Pounds.of(12);
  public static final Distance MAX_POSITION = Inches.of(9.879);
  public static final Distance MIN_POSITION = Inches.of(0);
  public static final Angle ANGLE = Degrees.of(-18);
  public static final Distance PINION_RADIUS = Inches.of(0.5);
  public static final int CANDI_DEVICE_ID = 40;
  public static final Distance POSITION_TOLERANCE = Inches.of(0.125);

  public static final TalonFXConfiguration MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(4.5))
          .withMotionMagic(
              new MotionMagicConfigs()
                  // fake numbers
                  .withMotionMagicCruiseVelocity(0.36)
                  .withMotionMagicAcceleration(0.72)
                  .withMotionMagicJerk(0))
          // fake numbers end here
          .withSlot0(
              new Slot0Configs().withKA(0.00).withKD(0.0).withKP(56.0).withKG(-0.0742).withKV(7))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(120))
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  public static final CANdiConfiguration CANDI_CONFIGURATION = new CANdiConfiguration();
}
