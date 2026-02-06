package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ClimbConstants{
    public static final double GEAR_RATIO = 10.0;
    public static final double MASS = 10; 
    public static final double DRUM_RADIUS = 1;
    public static final int CANDI_CAN_ID = 30;
    public static final int MOTOR_ID = 30;
    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration()
    .withMotorOutput(null)
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true))
    .withSoftwareLimitSwitch(
        new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitThreshold(0.0)
            .withForwardSoftLimitThreshold(10)
            .withReverseSoftLimitEnable(true))
    .withSlot0(new Slot0Configs()
        .withKP(2.4)
        .withKI(0.0)
        .withKD(0.0)
        .withKG(0.5)
        .withGravityType(GravityTypeValue.Elevator_Static))
    .withMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(80.0)
            .withMotionMagicAcceleration(160.0)
            .withMotionMagicJerk(1000.0));
}
