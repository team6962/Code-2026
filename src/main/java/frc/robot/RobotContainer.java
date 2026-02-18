// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.vision.AprilTagVision;
import com.team6962.lib.vision.SphereClumpLocalization;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.DriveStraightAuto;
import frc.robot.controls.TeleopControls;
import frc.robot.learnbot.LearnBotConstants;
import frc.robot.subsystems.hood.ShooterHood;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooterrollers.ShooterRollers;
import frc.robot.subsystems.turret.Turret;

public class RobotContainer {
  private final CommandSwerveDrive swerveDrive;
  private final TeleopControls teleopControls;
  private final Turret turret;
  private final DriveStraightAuto driveStraightAuto;
  private final ShooterHood shooterHood;
  private final SphereClumpLocalization fuelClumpLocalization;
  private final ShooterRollers shooterRollers;
  private final IntakeRollers intakeRollers;
  private final AprilTagVision aprilTagVision;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    LoggingUtil.logGitProperties();

    swerveDrive =
        new CommandSwerveDrive(Preferences.apply(LearnBotConstants.getDrivetrainConstants()));

    shooterHood = new ShooterHood();
    intakeRollers = new IntakeRollers();
    shooterRollers = new ShooterRollers();
    turret = new Turret();

    aprilTagVision =
        new AprilTagVision(swerveDrive, LearnBotConstants.getAprilTagVisionConstants());
    fuelClumpLocalization =
        new SphereClumpLocalization(swerveDrive, LearnBotConstants.getSphereCameraConstants());

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
    SmartDashboard.putData("Select Autonomous Routine", autoChooser);
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
}
