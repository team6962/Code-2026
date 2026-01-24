// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.swerve.CommandSwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.controls.TeleopControls;
import frc.robot.subsystems.TurretRotation;

public class RobotContainer {
  // private final CommandSwerveDrive swerveDrive;
  // private final TeleopControls teleopControls;
  private TurretRotation turretRotation;
  public RobotContainer() {
    LoggingUtil.logGitProperties();
    // swerveDrive = new CommandSwerveDrive(LearnBotConstants.getDrivetrainConstants());
    // teleopControls = new TeleopControls(this);
    // teleopControls.configureBindings();
    this.turretRotation = new TurretRotation();
  }

  public CommandSwerveDrive getSwerveDrive() {
    return null;
    // return swerveDrive;
  }

  public Command getAutonomousCommand() {
    return turretRotation.moveToleft();
  }

  public void latePeriodic() {
    // swerveDrive.latePeriodic();
  }
}