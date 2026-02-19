// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.vision.AprilTagVision;
import com.team6962.lib.vision.SphereClumpLocalization;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.DriveStraightAuto;
import frc.robot.constants.LearnBotConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.controls.TeleopControls;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooterrollers.ShooterRoller;

public class RobotContainer {
  private final RobotConstants constants;
  private final CommandSwerveDrive swerveDrive;
  private final TeleopControls teleopControls;
  private final DriveStraightAuto driveStraightAuto;
  private final SphereClumpLocalization fuelClumpLocalization;
  private final ShooterRoller shooterRoller;
  private final IntakeRollers intakeRollers;
  private final AprilTagVision aprilTagVision;

  public RobotContainer() {
    LoggingUtil.logGitProperties();

    constants = RobotConstants.generate();

    swerveDrive = new CommandSwerveDrive(constants.getDrivetrainConstants());

    intakeRollers = new IntakeRollers();

    aprilTagVision = new AprilTagVision(swerveDrive, constants.getAprilTagVisionConstants());
    fuelClumpLocalization =
        new SphereClumpLocalization(swerveDrive, constants.getSphereCameraConstants());

    teleopControls = new TeleopControls(this);
    teleopControls.configureBindings();

    driveStraightAuto = new DriveStraightAuto(this);
    shooterRoller = new ShooterRoller();
  }

  public RobotConstants getConstants() {
    return constants;
  }

  public CommandSwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public AprilTagVision getAprilTagVision() {
    return aprilTagVision;
  }

  public Command getAutonomousCommand() {
    return driveStraightAuto.getCommand();
  }

  public void latePeriodic() {
    swerveDrive.latePeriodic();
  }

  public SphereClumpLocalization getFuelLocalization() {
    return fuelClumpLocalization;
  }
}
