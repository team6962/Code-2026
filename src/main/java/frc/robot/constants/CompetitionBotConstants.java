package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
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
import com.team6962.lib.vision.AprilTagCameraConstants;
import com.team6962.lib.vision.AprilTagVisionConstants;
import com.team6962.lib.vision.SphereCameraConstants;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.photonvision.simulation.SimCameraProperties;

public class CompetitionBotConstants extends BaseRobotConstants {
  @Override
  public DrivetrainConstants getDrivetrainConstants() {
    DrivetrainConstants baseConstants = super.getDrivetrainConstants();

    TalonFXConfiguration baseDriveMotorConfig =
        new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(13)
                    .withMotionMagicAcceleration(20))
            .withSlot0(
                new Slot0Configs()
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withSupplyCurrentLimit(Amps.of(50)))
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
    mk4cDriveMotorConfig.Slot0.kP = 0.5;
    mk4cDriveMotorConfig.Slot0.kV = 0.708;
    mk4cDriveMotorConfig.Slot0.kS = 0.092619;
    mk4cDriveMotorConfig.Slot0.kA = 0.1255 / 2.0;

    TalonFXConfiguration mk4cSteerMotorConfig = baseSteerMotorConfig.clone();
    mk4cSteerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    mk4cSteerMotorConfig.Feedback.RotorToSensorRatio = 12.8;
    mk4cSteerMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(15)
            .withKD(0.5)
            .withKS(0.19246)
            .withKV(12.0 / (6000.0 / 60.0) * 12.8) // KV = gear ratio * peak voltage / free speed
            .withKA(0.068177); // SysId

    UniqueModuleConstants mk4cConstants =
        new UniqueModuleConstants()
            .withSteerGearReduction(12.8)
            .withSteerMotorConfig(mk4cSteerMotorConfig)
            .withSteerMomentOfInertia(KilogramSquareMeters.of(0.03))
            .withDriveMotorConfig(mk4cDriveMotorConfig);

    TalonFXConfiguration mk4nADriveMotorConfig = baseDriveMotorConfig.clone();
    mk4nADriveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    mk4nADriveMotorConfig.Slot0.kP = 0.5;
    mk4nADriveMotorConfig.Slot0.kV = 0.708;
    mk4nADriveMotorConfig.Slot0.kS = 0.13811;
    mk4nADriveMotorConfig.Slot0.kA = 0.18457 / 2.0;

    TalonFXConfiguration mk4nASteerMotorConfig = baseSteerMotorConfig.clone();
    mk4nASteerMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    mk4nASteerMotorConfig.Feedback.RotorToSensorRatio = 18.75;
    mk4nASteerMotorConfig.Slot0 =
        new Slot0Configs()
            .withKP(12.916)
            .withKD(0.40581)
            .withKS(0.25503)
            .withKV(12.0 / (6000.0 / 60.0) * 18.75) // KV = gear ratio * peak voltage / free speed
            .withKA(0.085981); // SysId

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
                .withWheelRadius(Inches.of(1.907879))) // Measured on 971 practice field
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
                .withTimesyncControlRequests(true)
                .withUseThreadedControlLoop(true)
                .withMinimizeLogging(true))
        .withDriving(
            baseConstants
                .Driving
                .clone()
                .withMaxLinearVelocity(MetersPerSecond.of(4))
                .withMaxLinearAcceleration(MetersPerSecondPerSecond.of(5))
                .withMaxAngularVelocity(RotationsPerSecond.of(1))
                .withMaxAngularAcceleration(RotationsPerSecondPerSecond.of(1))
                .withAutoLinearVelocity(MetersPerSecond.of(3.5))
                .withAutoLinearAcceleration(MetersPerSecondPerSecond.of(3))
                .withAutoAngularVelocity(RotationsPerSecond.of(1))
                .withAutoAngularAcceleration(RotationsPerSecondPerSecond.of(0.5))
                .withAutoLinearAccelerationScalar(0.06)
                .withAutoAngularAccelerationScalar(0.06)
                .withTranslationFeedbackKP(0.25) // Not tuned
                .withTranslationFeedbackKD(0.0) // Not tuned
                .withAngleFeedbackKP(0.5) // Not tuned
                .withAngleFeedbackKD(0.0)) // Not tuned
        .withDriveMotor(
            baseConstants
                .DriveMotor
                .clone()
                .withDeviceConfiguration(baseDriveMotorConfig)
                .withGearReduction(5.9)
                .withOutputType(ControlOutputType.Voltage)
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
                .withOutputType(ControlOutputType.Voltage)
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
    return super.getAprilTagVisionConstants()
        .withCameras(
            new AprilTagCameraConstants(
                "Monochrome-7",
                new Transform3d(
                    new Translation3d(
                        Inches.of(-12.866392).in(Meters),
                        Inches.of(-12.866926).in(Meters),
                        Inches.of(7.688516).in(Meters)),
                    new Rotation3d(0, -Math.PI / 6, -(3 * Math.PI) / 4))),
            new AprilTagCameraConstants(
                "Monochrome-9",
                new Transform3d(
                    new Translation3d(
                        Inches.of(-12.866392).in(Meters),
                        Inches.of(12.866926).in(Meters),
                        Inches.of(7.688516).in(Meters)),
                    new Rotation3d(0, -Math.PI / 6, (3 * Math.PI) / 4))),
            new AprilTagCameraConstants(
                "Monochrome-8",
                new Transform3d(
                    new Translation3d(
                        Inches.of(-10.293558).in(Meters),
                        Inches.of(-7.882).in(Meters),
                        Inches.of(20.601426).in(Meters)),
                    new Rotation3d(
                        Degrees.of(-18.224755).in(Radians),
                        Degrees.of(-26.25).in(Radians),
                        Math.PI / 4))),
            new AprilTagCameraConstants(
                "Monochrome-4",
                new Transform3d(
                    new Translation3d(
                        Inches.of(-10.293558).in(Meters),
                        Inches.of(-12.118).in(Meters),
                        Inches.of(20.601426).in(Meters)),
                    new Rotation3d(
                        Degrees.of(18.224755).in(Radians),
                        Degrees.of(-26.25).in(Radians),
                        -Math.PI / 4))))
        // Note that standard deviations are not fully tuned
        .withSingleTagStdDevs(VecBuilder.fill(20.0, 20.0, 20.0, 60.0))
        .withMultiTagStdDevs(VecBuilder.fill(0.03, 0.03, 0.03, 0.15))
        .withStdDevDistanceScalar(0.3)
        .withCameraSimProperties(
            new SimCameraProperties()
                .setCalibration(640, 480, Rotation2d.fromDegrees(60.54)) // needs to be checked
                .setCalibError(0.23, 0.0442) // needs to be checked
                .setFPS(50) // needs to be checked
                .setAvgLatencyMs(20) // needs to be checked
                .setLatencyStdDevMs(5)) // needs to be checked
        .withDrawWireframes(true)
        .withMinTagsForHeadingUpdateWhileEnabled(
            Integer.MAX_VALUE) // No heading updates from vision while enabled
        .withMinTagsForHeadingUpdateWhileDisabled(1);
  }

  @Override
  public SphereCameraConstants getSphereCameraConstants() {
    return super.getSphereCameraConstants()
        .withName("Color-3")
        .withClassId(0)
        .withFOVHeight(Rotation2d.fromDegrees(47.23))
        .withFOVWidth(Rotation2d.fromDegrees(60.48))
        .withCameraHeightPixels(600)
        .withCameraWidthPixels(800)
        .withFocalLengthX(686.17)
        .withFocalLengthY(686.21)
        .withMaxDetectionRange(
            Inches.of(426.708074544)) // Farthest distance the camera can detect a target
        .withSphereDiameter(Inches.of(5.91))
        .withMaxTargets(50) // Temporary value until we tune object detection
        .withRobotToCameraTransform(
            new Transform3d(
                new Translation3d(
                    Inches.of(12.6644).in(Meters), 0, 0), // Placeholder for real camera transform
                new Rotation3d(0, Degrees.of(25.2).in(Radians), 0)));
  }

  @Override
  public XBoxTeleopSwerveConstants getTeleopSwerveConstants() {
    return super.getTeleopSwerveConstants();
  }
}
