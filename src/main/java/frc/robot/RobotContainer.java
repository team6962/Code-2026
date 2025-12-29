// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.swerve.commands.DriveToStateCommand;
import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;
import com.team6962.lib.swerve.config.ControlMode;
import com.team6962.lib.swerve.config.ControlMode.OutputTypeValue;
import com.team6962.lib.swerve.config.ControlMode.PositionMotionProfileValue;
import com.team6962.lib.swerve.config.ControlMode.VelocityMotionProfileValue;
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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private CommandSwerveDrive swerveDrive;
  private XBoxTeleopSwerveCommand teleopSwerveCommand;

  public RobotContainer() {
    swerveDrive = new CommandSwerveDrive(
      new DrivetrainConstants()
        .withCANBusName("drivetrain")
        .withGyroscope(new GyroscopeConstants().withCANId(0))
        .withStructure(
          new StructureConstants()
            .withTrackWidth(Inches.of(22.75))
            .withWheelBase(Inches.of(22.75))
            .withRobotMass(Pounds.of(125))
            .withRobotMomentOfInertia(KilogramSquareMeters.of(3.15))
            .withWheelRadius(Inches.of(2))
        )
        .withSwerveModules(
          new SwerveModuleConstants[] {
            new SwerveModuleConstants()
              .withDriveMotorCANId(0)
              .withSteerMotorCANId(1)
              .withSteerEncoderCANId(0)
              .withSteerEncoderOffset(Degrees.of(0)),
            new SwerveModuleConstants()
              .withDriveMotorCANId(2)
              .withSteerMotorCANId(3)
              .withSteerEncoderCANId(1)
              .withSteerEncoderOffset(Degrees.of(0)),
            new SwerveModuleConstants()
              .withDriveMotorCANId(4)
              .withSteerMotorCANId(5)
              .withSteerEncoderCANId(2)
              .withSteerEncoderOffset(Degrees.of(0)),
            new SwerveModuleConstants()
              .withDriveMotorCANId(6)
              .withSteerMotorCANId(7)
              .withSteerEncoderCANId(3)
              .withSteerEncoderOffset(Degrees.of(0))
          }
        )
        .withTiming(
          new TimingConstants()
            .withControlLoopFrequency(Hertz.of(50))
            .withUseThreadedControlLoop(false)
            .withSignalUpdateRate(Hertz.of(100))
        )
        .withDriving(
          new DrivingConstants()
            .withMaxLinearVelocity(MetersPerSecond.of(4))
            .withMaxLinearAcceleration(MetersPerSecondPerSecond.of(5))
            .withMaxAngularVelocity(RotationsPerSecond.of(1))
            .withMaxAngularAcceleration(RotationsPerSecondPerSecond.of(1))
            .withTranslationFeedbackKP(1.0)
            .withAngleFeedbackKP(1.0)
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
                    .withKV(0.118 * 5.9)
                    .withKA(0.003933 / 5.9)
                    .withKS(0.17)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                )
                .withCurrentLimits(
                  new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(false)
                )
            )
            .withGearReduction(5.9)
            .withVelocityControl(new ControlMode.Velocity(
              VelocityMotionProfileValue.Trapezoidal,
              OutputTypeValue.Voltage
            ))
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
                    // .withInverted(InvertedValue.Clockwise_Positive)
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
                    .withKS(0.282)
                    .withKV(2.6)
                    .withKA(0.03)
                    .withKP(18.592)
                    .withKI(0.0)
                    .withKD(0.972)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                )
            )
            .withGearReduction(150.0 / 7.0)
            .withPositionControl(new ControlMode.Position(
              PositionMotionProfileValue.Exponential,
              OutputTypeValue.Voltage
            ))
            .withPositionSlot(0)
            .withSimulatedMotor(DCMotor.getKrakenX60(1))
            .withSimulatedMomentOfInertia(KilogramSquareMeters.of(0.000184))
        )
        .withSteerEncoder(
          new SteerEncoderConstants()
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
    ));
  }

  public void latePeriodic() {
    swerveDrive.latePeriodic();
  }
}
