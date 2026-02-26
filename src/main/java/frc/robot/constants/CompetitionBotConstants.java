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
import com.team6962.lib.swerve.config.UniqueModuleConstants;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import com.team6962.lib.vision.AprilTagVisionConstants;
import com.team6962.lib.vision.SphereCameraConstants;

import edu.wpi.first.math.system.plant.DCMotor;

public class CompetitionBotConstants extends BaseRobotConstants {
  @Override
  public DrivetrainConstants getDrivetrainConstants() {
    DrivetrainConstants baseConstants = super.getDrivetrainConstants();

    TalonFXConfiguration baseDriveMotorConfig =
        new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(14.4)
                    .withMotionMagicAcceleration(21.2))
            .withSlot0(
                new Slot0Configs()
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withSupplyCurrentLimit(Amps.of(80)))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    TalonFXConfiguration baseSteerMotorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withSupplyCurrentLimit(Amps.of(60)))
            .withMotionMagic(
                new MotionMagicConfigs().withMotionMagicExpo_kV(2).withMotionMagicExpo_kA(0.25));

    // MODULE PROPERTIES (TODO: Check in CAD)
    //                        MK4c MK4nA MK4nB
    // Steer Motor Inversion: CCW  CW    CW
    // Steer Motor Ratio:     12.8 18.75 18.75
    // Drive Motor Inversion: CCW  CW    CCW
    // MOI:                   0.000861933929 0.0012006085 0.0012006085

    TalonFXConfiguration mk4cDriveMotorConfig = baseDriveMotorConfig.clone();
    mk4cDriveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    mk4cDriveMotorConfig.Slot0.kP = 2.0;
    mk4cDriveMotorConfig.Slot0.kV = 0.71;
    mk4cDriveMotorConfig.Slot0.kS = 0.144;

    TalonFXConfiguration mk4cSteerMotorConfig = baseSteerMotorConfig.clone();
    mk4cSteerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    mk4cSteerMotorConfig.Feedback.RotorToSensorRatio = 12.8;
    mk4cSteerMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(15)
            .withKD(0.5)
            .withKS(0.325)
            .withKV(12.0 / (5800.0 / 60.0) * 12.8) // KV = gear ratio * peak voltage / free speed
            .withKA(0.034); // KA = MOI * peak voltage / gear ratio / stall torque

    UniqueModuleConstants mk4cConstants =
        new UniqueModuleConstants()
            .withSteerGearReduction(12.8)
            .withSteerMotorConfig(mk4cSteerMotorConfig)
            .withSteerMomentOfInertia(KilogramSquareMeters.of(0.03))
            .withDriveMotorConfig(mk4cDriveMotorConfig);

    TalonFXConfiguration mk4nADriveMotorConfig = baseDriveMotorConfig.clone();
    mk4nADriveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    mk4nADriveMotorConfig.Slot0.kP = 2.0;
    mk4nADriveMotorConfig.Slot0.kV = 0.71;
    mk4nADriveMotorConfig.Slot0.kS = 0.144;

    TalonFXConfiguration mk4nASteerMotorConfig = baseSteerMotorConfig.clone();
    mk4nASteerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    mk4nASteerMotorConfig.Feedback.RotorToSensorRatio = 18.75;
    mk4nASteerMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(25)
            .withKD(1.25)
            .withKS(0.24)
            .withKV(12.0 / (5800.0 / 60.0) * 18.75) // KV = gear ratio * peak voltage / free speed
            .withKA(0.08); // KA = MOI * peak voltage / gear ratio / stall torque

    UniqueModuleConstants mk4nAConstants =
        new UniqueModuleConstants()
            .withSteerGearReduction(18.75)
            .withSteerMotorConfig(mk4nASteerMotorConfig)
            .withSteerMomentOfInertia(KilogramSquareMeters.of(0.03))
            .withDriveMotorConfig(mk4nADriveMotorConfig);

    TalonFXConfiguration mk4nBDriveMotorConfig = mk4nADriveMotorConfig.clone();
    mk4nBDriveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXConfiguration mk4nBSteerMotorConfig = mk4nASteerMotorConfig.clone();
    mk4nBSteerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    UniqueModuleConstants mk4nBConstants =
        new UniqueModuleConstants()
            .withSteerGearReduction(mk4nAConstants.SteerGearReduction)
            .withSteerMotorConfig(mk4nBSteerMotorConfig)
            .withSteerMomentOfInertia(mk4nAConstants.SteerMomentOfInertia)
            .withDriveMotorConfig(mk4nBDriveMotorConfig);

    return baseConstants
        .withCANBusName("drivetrain")
        .withGyroscope(new GyroscopeConstants().withCANId(10))
        .withStructure(
            baseConstants
                .Structure
                .clone()
                .withTrackWidth(Inches.of(21.75))
                .withWheelBase(Inches.of(21.75))
                .withRobotMass(Pounds.of(135)) // Estimated
                .withRobotMomentOfInertia(KilogramSquareMeters.of(6)) // Estimated
                .withWheelRadius(Inches.of(1.95265))) // Measured with used wheels
        .withSwerveModules(
            new SwerveModuleConstants[] {
              new SwerveModuleConstants()
                  .withDriveMotorCANId(10)
                  .withSteerMotorCANId(11)
                  .withSteerEncoderCANId(10)
                  .withSteerEncoderOffset(Radians.of(-1.9325))
                  .withUniqueModuleConstants(mk4nBConstants),
              new SwerveModuleConstants()
                  .withDriveMotorCANId(12)
                  .withSteerMotorCANId(13)
                  .withSteerEncoderCANId(11)
                  .withSteerEncoderOffset(Radians.of(1.9017))
                  .withUniqueModuleConstants(mk4nAConstants),
              new SwerveModuleConstants()
                  .withDriveMotorCANId(14)
                  .withSteerMotorCANId(15)
                  .withSteerEncoderCANId(12)
                  .withSteerEncoderOffset(Radians.of(-2.7078))
                  .withUniqueModuleConstants(mk4cConstants),
              new SwerveModuleConstants()
                  .withDriveMotorCANId(16)
                  .withSteerMotorCANId(17)
                  .withSteerEncoderCANId(13)
                  .withSteerEncoderOffset(Radians.of(0.2274))
                  .withUniqueModuleConstants(mk4cConstants)
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
                .withDeviceConfiguration(baseDriveMotorConfig)
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
                    baseSteerMotorConfig) // Unused because all modules are given unique constants
                .withGearReduction(12.8) // Unused because all modules are given unique constants
                .withOutputType(ControlOutputType.VoltageFOC)
                .withPositionControlMotionProfile(PositionMotionProfileType.Exponential)
                .withPositionSlot(0)
                .withSimulatedMotor(DCMotor.getKrakenX60Foc(1))
                .withSimulatedMomentOfInertia(
                    KilogramSquareMeters.of(
                        0.03))) // Unused because all modules are given unique constants
        .withSteerEncoder(
            super.getDrivetrainConstants()
                .SteerEncoder
                .clone()
                .withDataFusion(DataFusionMethod.Remote));
  }

  @Override
  public AprilTagVisionConstants getAprilTagVisionConstants() {
    return super.getAprilTagVisionConstants();
  }

  @Override
  public SphereCameraConstants getSphereCameraConstants() {
    return super.getSphereCameraConstants();
    // return super.getSphereCameraConstants()
    //     .withName("Color-2")
    //     .withClassId(0)
    //     .withFOVHeight(Rotation2d.fromDegrees(48.9))
    //     .withFOVWidth(Rotation2d.fromDegrees(70))
    //     .withCameraHeightPixels(800)
    //     .withCameraWidthPixels(1280)
    //     .withFocalLengthX(907.41)
    //     .withFocalLengthY(907.64)
    //     .withMaxDetectionRange(Meters.of(18.37)) // diagonal length of the field
    //     .withSphereDiameter(Inches.of(5.91))
    //     .withMaxTargets(50) // Temporary value until we tune object detection on the practice field
    //     .withRobotToCameraTransform(
    //         new Transform3d(
    //             new Translation3d(Inches.of(16.25).in(Meters), 0, Inches.of(20.0).in(Meters)),
    //             new Rotation3d(0, Math.PI / 6, 0)));
  }

  @Override
  public XBoxTeleopSwerveConstants getTeleopSwerveConstants() {
    return super.getTeleopSwerveConstants();
  }
}
