package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;

public class ClimbConstants {
  public static final double GEAR_RATIO = 20.0;
  public static final Mass MASS = Pounds.of(2.1);
  public static final Distance DRUM_RADIUS = Inches.of(0.375);
  public static final int CANDI_CAN_ID = 40;
  public static final int MOTOR_ID = 30;
  public static final String CANBUS_NAME = "subsystems";

  public static final Distance MIN_HEIGHT = Inches.of(0);
  public static final Distance MAX_HEIGHT = Inches.of(25.045099);
  public static final Distance PULL_UP_HEIGHT = Inches.of(0);

  public static final Voltage FINE_CONTROL_VOLTAGE = Volts.of(0.5);

  public static final LinearAcceleration CONSTANT_ACCELERATION = MetersPerSecondPerSecond.of(-9.81);

  public static final TalonFXConfiguration MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withMotorOutput(new MotorOutputConfigs())
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(120)
                  .withSupplyCurrentLimit(80))
          .withSoftwareLimitSwitch(
              new SoftwareLimitSwitchConfigs()
                  .withForwardSoftLimitEnable(true)
                  .withReverseSoftLimitThreshold(0.0)
                  .withForwardSoftLimitThreshold(MAX_HEIGHT.in(Meters))
                  .withReverseSoftLimitEnable(true))
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio(GEAR_RATIO / DRUM_RADIUS.in(Meters) / (2 * Math.PI)))
          .withSlot0(
              new Slot0Configs()
                  .withKP(2.4)
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKG(0.5)
                  .withGravityType(GravityTypeValue.Elevator_Static))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(1.0)
                  .withMotionMagicAcceleration(1.0));
  public static final CANdiConfiguration CANDI_CONFIGURATION = new CANdiConfiguration();
}
