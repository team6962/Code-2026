// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.phoenix.control.ControlOutputType;
import com.team6962.lib.phoenix.control.PositionMotionProfileType;
import com.team6962.lib.phoenix.control.VelocityMotionProfileType;
import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.swerve.commands.DriveToStateCommand;
import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;
import com.team6962.lib.swerve.config.DriveMotorConstants;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.DrivingConstants;
import com.team6962.lib.swerve.config.GyroscopeConstants;
import com.team6962.lib.swerve.config.SteerEncoderConstants;
import com.team6962.lib.swerve.config.SteerMotorConstants;
import com.team6962.lib.swerve.config.StructureConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants;
import com.team6962.lib.swerve.config.TimingConstants;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import com.team6962.lib.swerve.config.SteerEncoderConstants.DataFusionMethod;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private CommandSwerveDrive swerveDrive;
  private XBoxTeleopSwerveCommand teleopSwerveCommand;

  public RobotContainer() {
    LoggingUtil.logGitProperties();

    swerveDrive = new CommandSwerveDrive(
      new DrivetrainConstants()
        .withCANBusName("drivetrain")
        .withGyroscope(new GyroscopeConstants().withCANId(0))
        .withStructure(
          new StructureConstants()
            .withTrackWidth(Inches.of(22.75))
            .withWheelBase(Inches.of(22.75))
            .withRobotMass(Pounds.of(40))
            .withRobotMomentOfInertia(KilogramSquareMeters.of(1.5))
            .withWheelRadius(Inches.of(3.9053 / 2))
        )
        .withSwerveModules(
          new SwerveModuleConstants[] {
            new SwerveModuleConstants()
              .withDriveMotorCANId(0)
              .withSteerMotorCANId(2)
              .withSteerEncoderCANId(2)
              .withSteerEncoderOffset(Radians.of(0.05)),
            new SwerveModuleConstants()
              .withDriveMotorCANId(3)
              .withSteerMotorCANId(1)
              .withSteerEncoderCANId(0)
              .withSteerEncoderOffset(Radians.of(0.049 + Math.PI)),
            new SwerveModuleConstants()
              .withDriveMotorCANId(5)
              .withSteerMotorCANId(7)
              .withSteerEncoderCANId(1)
              .withSteerEncoderOffset(Radians.of(2.4)),
            new SwerveModuleConstants()
              .withDriveMotorCANId(4)
              .withSteerMotorCANId(6)
              .withSteerEncoderCANId(3)
              .withSteerEncoderOffset(Radians.of(0.322 + Math.PI))
          }
        )
        .withTiming(
          new TimingConstants()
            .withControlLoopFrequency(Hertz.of(100))
            .withSignalUpdateRate(Hertz.of(100))
            .withTimesyncControlRequests(false)
            .withUseThreadedControlLoop(true)
        )
        .withDriving(
          new DrivingConstants()
            .withMaxLinearVelocity(MetersPerSecond.of(4))
            .withMaxLinearAcceleration(MetersPerSecondPerSecond.of(5))
            .withMaxAngularVelocity(RotationsPerSecond.of(1))
            .withMaxAngularAcceleration(RotationsPerSecondPerSecond.of(1))
            .withTranslationFeedbackKP(0.5)
            .withTranslationFeedbackKD(0.1)
            .withAngleFeedbackKP(0.25)
            .withAngleFeedbackKD(0.05)
        )
        .withDriveMotor(
          new DriveMotorConstants()
            .withDeviceConfiguration(
              new TalonFXConfiguration()
                .withMotionMagic(
                  new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(85 / 5.9)
                    .withMotionMagicAcceleration(125 / 5.9)
                )
                .withSlot0(
                  new Slot0Configs()
                    .withKP(0.1 / 5.9)
                    .withKI(0.01 / 5.9)
                    .withKD(0.01 / 5.9)
                    .withKV(0.708) // theoretical value for trapezoidal commutation
                    .withKA(0.003933 / 5.9)
                    .withKS(0.17 / 5.9)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                )
                .withCurrentLimits(
                  new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(false)
                )
                .withMotorOutput(
                  new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                )
            )
            .withGearReduction(5.9)
            .withOutputType(ControlOutputType.Voltage)
            .withVelocityControlMotionProfile(VelocityMotionProfileType.Trapezoidal)
            .withVelocitySlot(0)
            .withSimulatedMotor(DCMotor.getKrakenX60(1))
            .withSimulatedMomentOfInertia(KilogramSquareMeters.of(0.000307))
            .withMaxVelocity(MetersPerSecond.of(4.474))
        )
        .withSteerMotor(
          new SteerMotorConstants()
            .withDeviceConfiguration(
              new TalonFXConfiguration()
                .withMotorOutput(
                  new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
                )
                .withCurrentLimits(
                  new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Amps.of(60))
                    .withSupplyCurrentLimitEnable(true)
                )
                .withMotionMagic(
                  new MotionMagicConfigs()
                    .withMotionMagicExpo_kV(2.7626 * 0.75)
                    .withMotionMagicExpo_kA(0.3453 * 0.75)
                )
                .withSlot0(
                  new Slot0Configs()
                    .withKS(0.15)
                    .withKV(2.5714) // theoretical value for trapezoidal commutation
                    .withKA(0.03)
                    .withKP(18.592)
                    .withKI(0.0)
                    .withKD(0.972)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                )
            )
            .withGearReduction(150.0 / 7.0)
            .withOutputType(ControlOutputType.Voltage)
            .withPositionControlMotionProfile(PositionMotionProfileType.Exponential)
            .withPositionSlot(0)
            .withSimulatedMotor(DCMotor.getKrakenX60(1))
            .withSimulatedMomentOfInertia(KilogramSquareMeters.of(0.000184))
        )
        .withSteerEncoder(
          new SteerEncoderConstants()
            .withDataFusion(DataFusionMethod.Remote)
        )
    );

    Trigger teleopTrigger = new Trigger(() -> RobotState.isTeleop() && RobotState.isEnabled());

    teleopSwerveCommand = new XBoxTeleopSwerveCommand(
      swerveDrive,
      new XBoxTeleopSwerveConstants()
    );

    teleopTrigger.whileTrue(teleopSwerveCommand);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return new DriveToStateCommand(swerveDrive, new DriveToStateCommand.State(
      new Translation2d(1, 1),
      new TranslationalVelocity(),
      Radians.of(0),
      RadiansPerSecond.of(0)
    )).withFinishWhenReached(false);
  }

  public void latePeriodic() {
    swerveDrive.latePeriodic();
  }
}
