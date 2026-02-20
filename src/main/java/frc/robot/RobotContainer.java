// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.vision.AprilTagVision;
import com.team6962.lib.vision.SphereClumpLocalization;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.DriveStraightAuto;
import frc.robot.constants.RobotConstants;
import frc.robot.controls.TeleopControls;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.hood.ShooterHood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intakeextension.IntakeExtension;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooterrollers.ShooterRollers;
import frc.robot.subsystems.turret.Turret;

public class RobotContainer {
  private final RobotConstants constants;
  private final CommandSwerveDrive swerveDrive;
  private final TeleopControls teleopControls;
  private final Turret turret;
  private final DriveStraightAuto driveStraightAuto;
  private final IntakeExtension intakeExtension;
  private final ShooterHood shooterHood;
  private final SphereClumpLocalization fuelClumpLocalization;
  private final ShooterRollers shooterRollers;
  private final IntakeRollers intakeRollers;
  private final AprilTagVision aprilTagVision;
  private final Climb climb;
  private final Hopper hopper;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    LoggingUtil.logGitProperties();

    constants = RobotConstants.generate();

    swerveDrive = new CommandSwerveDrive(constants.getDrivetrainConstants());

    climb = new Climb();
    shooterHood = new ShooterHood();
    intakeRollers = new IntakeRollers();
    shooterRollers = new ShooterRollers();
    turret = new Turret();
    intakeExtension = new IntakeExtension();
    hopper = new Hopper();

    aprilTagVision = new AprilTagVision(swerveDrive, constants.getAprilTagVisionConstants());
    fuelClumpLocalization =
        new SphereClumpLocalization(swerveDrive, constants.getSphereCameraConstants());

    teleopControls = new TeleopControls(this);
    teleopControls.configureBindings();

    driveStraightAuto = new DriveStraightAuto(this);

    configureAutonomousChooser();
  }

  private void configureAutonomousChooser() {
    // Set "Do Nothing" as the default option
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    // Add the Drive Straight auto as an optional selection
    autoChooser.addOption("Drive Straight", driveStraightAuto.getCommand());

    if (RobotBase.isSimulation()) {
      autoChooser.addOption(
          "Test Drive To Pose",
          swerveDrive.driveTo(
              new Pose2d(10, 5, Rotation2d.fromDegrees(0)), new ChassisSpeeds(-2, 2, 0)));

      autoChooser.addOption(
          "Test Drive To Pose with Final Velocity",
          swerveDrive
              .driveTo(new Pose2d(10, 5, Rotation2d.fromDegrees(0)), new ChassisSpeeds(-2, 2, 0))
              .andThen(swerveDrive.drive(new ChassisSpeeds(-2, 2, 0))));
    }

    autoChooser.addOption("Calibrate Wheel Size", swerveDrive.calibrateWheelSize());

    SmartDashboard.putData("Select Autonomous Routine", autoChooser);
  }

  public RobotConstants getConstants() {
    return constants;
  }

  public IntakeExtension getIntakeExtension() {
    return intakeExtension;
  }

  public CommandSwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Turret getTurret() {
    return turret;
  }

  public AprilTagVision getAprilTagVision() {
    return aprilTagVision;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void latePeriodic() {
    swerveDrive.latePeriodic();
  }

  public SphereClumpLocalization getFuelLocalization() {
    return fuelClumpLocalization;
  }

  public ShooterHood getShooterHood() {
    return shooterHood;
  }

  public IntakeRollers getIntakeRollers() {
    return intakeRollers;
  }

  public ShooterRollers getShooterRollers() {
    return shooterRollers;
  }

  public Climb getClimb() {
    return climb;
  }

  public Hopper getHopper() {
    return hopper;
  }
}
