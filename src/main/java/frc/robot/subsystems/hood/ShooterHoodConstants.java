package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

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
    public static final Angle MIN_ANGLE = Degrees.of(-90);
    public static final Angle MAX_ANGLE = Degrees.of(90);
    public static final Angle INITIAL_ANGLE = Degrees.of(0);
    public static final Angle END_TOLERANCE = Degrees.of(3);
    public static final Angle HOLD_TOLERANCE = Degrees.of(5);
    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration()
        .withFeedback(
            new FeedbackConfigs()
                .withRotorToSensorRatio(42.0)
                .withSensorToMechanismRatio(1.0)
        )
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(10)
                .withMotionMagicAcceleration(5)
                .withMotionMagicJerk(0)
        )
        .withSlot0(
            new Slot0Configs()
                .withKP(0.5)
                .withKD(0.1)
                .withKS(0.150)
                .withKG(0.0)
                .withKV(2.571)
                .withKA(5)
                .withGravityType(GravityTypeValue.Arm_Cosine)
        );
}
