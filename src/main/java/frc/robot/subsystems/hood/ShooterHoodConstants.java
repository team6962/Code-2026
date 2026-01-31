package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;

public final class ShooterHoodConstants {
    public static final int MOTOR_CAN_ID = 20;
    public static final DCMotor MOTOR_PHYSICS = DCMotor.getKrakenX60Foc(1);

    public static final double MOI = 0.04942;

    public static final Angle MIN_ANGLE = Degrees.of(-90);
    public static final Angle MAX_ANGLE = Degrees.of(90);
    public static final Angle INITIAL_ANGLE = Degrees.of(0);

    public static final Angle END_TOLERANCE = Degrees.of(3);
    public static final Angle HOLD_TOLERANCE = Degrees.of(5);

    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration()
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(176/3)
        )
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(1)
                .withMotionMagicAcceleration(1)
                .withMotionMagicJerk(0)
        )
        .withSlot0(
            new Slot0Configs()
                .withKP(0)
                .withKD(0)
                .withKS(0)
                .withKG(0.128)
                .withKV(7.283)
                .withKA(0.001)
                .withGravityType(GravityTypeValue.Arm_Cosine)
        )
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(false)
                .withSupplyCurrentLimit(Amps.of(60))
                .withSupplyCurrentLimitEnable(true)
        );
}
