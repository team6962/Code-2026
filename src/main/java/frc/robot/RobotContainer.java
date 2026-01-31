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
import frc.robot.controls.TeleopControls;
import frc.robot.learnbot.LearnBotConstants;

public class RobotContainer {
  private final CommandSwerveDrive swerveDrive;
  private final TeleopControls teleopControls;
  private final DriveStraightAuto driveStraightAuto;
  private final AprilTagVision aprilTagVision;
  private final SphereClumpLocalization fuelClumpLocalization;
  public RobotContainer() {
    LoggingUtil.logGitProperties();

    swerveDrive =
        new CommandSwerveDrive(Preferences.apply(LearnBotConstants.getDrivetrainConstants()));

    aprilTagVision =
        new AprilTagVision(swerveDrive, LearnBotConstants.getAprilTagVisionConstants());
    fuelClumpLocalization = new SphereClumpLocalization(swerveDrive, LearnBotConstants.getSphereCameraConstants());
    teleopControls = new TeleopControls(this);
    teleopControls.configureBindings();

    driveStraightAuto = new DriveStraightAuto(this);
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
