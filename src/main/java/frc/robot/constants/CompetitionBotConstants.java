package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team6962.lib.phoenix.control.ControlOutputType;
import com.team6962.lib.phoenix.control.PositionMotionProfileType;
import com.team6962.lib.phoenix.control.VelocityMotionProfileType;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.GyroscopeConstants;
import com.team6962.lib.swerve.config.SteerEncoderConstants.DataFusionMethod;
import com.team6962.lib.swerve.config.SwerveModuleConstants;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import com.team6962.lib.vision.AprilTagVisionConstants;
import com.team6962.lib.vision.SphereCameraConstants;
import edu.wpi.first.math.system.plant.DCMotor;

public class CompetitionBotConstants extends BaseRobotConstants {
  @Override
  public DrivetrainConstants getDrivetrainConstants() {
    DrivetrainConstants baseConstants = super.getDrivetrainConstants();

    return baseConstants
        .withCANBusName("drivetrain")
        .withGyroscope(new GyroscopeConstants().withCANId(10))
        .withStructure(
            baseConstants
                .Structure
                .clone()
                .withTrackWidth(Inches.of(21.75))
                .withWheelBase(Inches.of(21.75))
                .withRobotMass(Pounds.of(120)) // Estimated from CAD
                .withRobotMomentOfInertia(KilogramSquareMeters.of(5.785)) // Estimated from CAD
                .withWheelRadius(Inches.of(3.9053 / 2))) // Measured with used wheels
        .withSwerveModules(
            new SwerveModuleConstants[] {
              new SwerveModuleConstants()
                  .withDriveMotorCANId(10)
                  .withSteerMotorCANId(11)
                  .withSteerEncoderCANId(10)
                  .withSteerEncoderOffset(Radians.of(0)), // Not tuned
              new SwerveModuleConstants()
                  .withDriveMotorCANId(12)
                  .withSteerMotorCANId(13)
                  .withSteerEncoderCANId(11)
                  .withSteerEncoderOffset(Radians.of(0)), // Not tuned
              new SwerveModuleConstants()
                  .withDriveMotorCANId(14)
                  .withSteerMotorCANId(15)
                  .withSteerEncoderCANId(12)
                  .withSteerEncoderOffset(Radians.of(0)), // Not tuned
              new SwerveModuleConstants()
                  .withDriveMotorCANId(16)
                  .withSteerMotorCANId(17)
                  .withSteerEncoderCANId(13)
                  .withSteerEncoderOffset(Radians.of(0)) // Not tuned
            })
        .withTiming(
            baseConstants
                .Timing
                .clone()
                .withControlLoopFrequency(Hertz.of(100))
                .withSignalUpdateRate(Hertz.of(100))
                .withTimesyncControlRequests(false)
                .withUseThreadedControlLoop(true))
        .withDriving(
            baseConstants
                .Driving
                .clone()
                .withMaxLinearVelocity(MetersPerSecond.of(4)) // Not tuned
                .withMaxLinearAcceleration(MetersPerSecondPerSecond.of(5)) // Not tuned
                .withMaxAngularVelocity(RotationsPerSecond.of(1)) // Not tuned
                .withMaxAngularAcceleration(RotationsPerSecondPerSecond.of(1)) // Not tuned
                .withTranslationFeedbackKP(0.5) // Not tuned
                .withTranslationFeedbackKD(0.1) // Not tuned
                .withAngleFeedbackKP(0.25) // Not tuned
                .withAngleFeedbackKD(0.05)) // Not tuned
        .withDriveMotor(
            baseConstants
                .DriveMotor
                .clone()
                .withDeviceConfiguration(
                    new TalonFXConfiguration()
                        .withMotionMagic(
                            new MotionMagicConfigs()
                                .withMotionMagicCruiseVelocity(14.4)
                                .withMotionMagicAcceleration(21.2))
                        .withSlot0(
                            new Slot0Configs()
                                .withKP(0.017)
                                .withKI(0.0017)
                                .withKD(0.0017)
                                .withKV(0.7324)
                                .withKA(0.00067)
                                .withKS(0.0288)
                                .withStaticFeedforwardSign(
                                    StaticFeedforwardSignValue.UseVelocitySign))
                        .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(120))
                                .withSupplyCurrentLimit(Amps.of(80)))
                        .withMotorOutput(
                            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)))
                .withGearReduction(5.9)
                .withOutputType(ControlOutputType.VoltageFOC)
                .withVelocityControlMotionProfile(VelocityMotionProfileType.Trapezoidal)
                .withVelocitySlot(0)
                .withSimulatedMotor(DCMotor.getKrakenX60Foc(1))
                .withSimulatedMomentOfInertia(KilogramSquareMeters.of(0.000307))
                .withMaxVelocity(MetersPerSecond.of(4.474)))
        .withSteerMotor(
            baseConstants
                .SteerMotor
                .clone()
                .withDeviceConfiguration(
                    new TalonFXConfiguration()
                        .withMotorOutput(
                            new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                        .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(Amps.of(120))
                                .withSupplyCurrentLimit(Amps.of(60)))
                        .withMotionMagic(
                            new MotionMagicConfigs()
                                .withMotionMagicExpo_kV(2.072)
                                .withMotionMagicExpo_kA(0.259))
                        .withSlot0(
                            new Slot0Configs()
                                .withKS(0.15)
                                .withKV(2.66)
                                .withKA(0.03)
                                .withKP(18.592)
                                .withKI(0.0)
                                .withKD(0.972)
                                .withStaticFeedforwardSign(
                                    StaticFeedforwardSignValue.UseVelocitySign)))
                .withGearReduction(150.0 / 7.0)
                .withOutputType(ControlOutputType.VoltageFOC)
                .withPositionControlMotionProfile(PositionMotionProfileType.Exponential)
                .withPositionSlot(0)
                .withSimulatedMotor(DCMotor.getKrakenX60Foc(1))
                .withSimulatedMomentOfInertia(KilogramSquareMeters.of(0.000184)))
        .withSteerEncoder(
            super.getDrivetrainConstants()
                .SteerEncoder
                .clone()
                .withDataFusion(DataFusionMethod.Fused));
  }

  @Override
  public AprilTagVisionConstants getAprilTagVisionConstants() {
    return super.getAprilTagVisionConstants();
  }

  @Override
  public SphereCameraConstants getSphereCameraConstants() {
    return super.getSphereCameraConstants();
  }

  @Override
  public XBoxTeleopSwerveConstants getTeleopSwerveConstants() {
    return super.getTeleopSwerveConstants();
  }

  @Override
  public EnabledSystems getEnabledSystems() {
    return super.getEnabledSystems();
  }
}
