package frc.robot.subsystems.intakeextension;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class IntakeExtensionConstants {
  public static final int MOTOR_CAN_ID = 40;
  public static final DCMotor MOTOR_PHYSICS = DCMotor.getKrakenX60Foc(1);
  public static final Mass MOVING_MASS = Pounds.of(11);
  public static final MomentOfInertia MOMENT_OF_INERTIA =
      KilogramSquareMeters.of(5); // dummy number.
  public static final Distance MAX_POSITION = Inches.of(10); // dummy number.
  public static final Distance MIN_POSITION = Inches.of(0); // dummy number.

  public static final TalonFXConfiguration MOTOR_CONFIGURATION =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(4.5))
          .withMotionMagic(
              new MotionMagicConfigs()
                  // fake numbers
                  .withMotionMagicCruiseVelocity(1)
                  .withMotionMagicAcceleration(0.2)
                  .withMotionMagicJerk(0))
          // fake numbers end here
          .withSlot0(new Slot0Configs().withKP(0.335))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(120))
                  .withSupplyCurrentLimit(Amps.of(60))
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimitEnable(true));
}
