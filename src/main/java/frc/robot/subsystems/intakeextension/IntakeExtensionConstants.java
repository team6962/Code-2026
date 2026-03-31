package frc.robot.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeExtensionConstants {

  public static final int MOTOR_CAN_ID = 40;
  public static final DCMotor MOTOR_PHYSICS = DCMotor.getKrakenX44Foc(1);
  public static final Mass MOVING_MASS = Pounds.of(12);
  public static final Distance MAX_POSITION = Inches.of(10.1);
  public static final Distance MIN_POSITION = Inches.of(0);
  public static final Distance RETRACT_POSITION = Inches.of(3);
  public static final Angle ANGLE = Degrees.of(-18.5);
  public static final Distance PINION_RADIUS = Inches.of(0.75);
  public static final int CANDI_DEVICE_ID = 20;
  public static final Distance POSITION_TOLERANCE = Inches.of(0.125);
  public static final Voltage FINE_CONTROL_VOLTAGE = Volts.of(1.0);
  public static final double GEAR_RATIO = 7.2;

  public static final TalonFXConfiguration MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio(
                      GEAR_RATIO / PINION_RADIUS.in(Meters) / (2 * Math.PI)))
          .withMotionMagic(
              new MotionMagicConfigs()
                  // Not tuned
                  .withMotionMagicCruiseVelocity(20.0)
                  .withMotionMagicAcceleration(12.0)
                  .withMotionMagicJerk(0))
          .withSlot0(
              new Slot0Configs()
                  .withKA(0.035)
                  .withKD(0.0)
                  .withKP(100.0)
                  .withKG(-0.045)
                  .withKV(5.878)
                  .withKS(0.315)
                  .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast));

  public static final CANdiConfiguration CANDI_CONFIGURATION = new CANdiConfiguration();
}
