package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;

public class ClimbConstants{
    public static final double GEAR_RATIO = 10.0;
    public static final Mass MASS = Kilograms.of(10); //needs to be updated. idk what the value should be
    public static final Distance DRUM_RADIUS = Inches.of(1);
    public static final int CANDI_CAN_ID = 30;
    public static final int MOTOR_ID = 30;

    // heights need to be updated to be real values
    public static final Distance MIN_HEIGHT = Inches.of(0);
    public static final Distance MAX_HEIGHT = Inches.of(72);

    public static final LinearAcceleration CONSTANT_ACCELERATION = MetersPerSecondPerSecond.of(1.0); //this number needs to be changed
    
    public static final TalonFXConfiguration MOTOR_CONFIGURATION = new TalonFXConfiguration()
    .withMotorOutput(null)
    .withCurrentLimits(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true))
    .withSoftwareLimitSwitch(
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(0.0)
            .withForwardSoftLimitThreshold(8) // change to elevator max height
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

    public static final CANdiConfiguration CANDI_CONFIGURATION = new CANdiConfiguration();
}
