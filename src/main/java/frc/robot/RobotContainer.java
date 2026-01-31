// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.swerve.CommandSwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.DriveStraightAuto;
import frc.robot.controls.TeleopControls;
import frc.robot.learnbot.LearnBotConstants;
import frc.robot.subsystems.shooterrollers.ShooterRoller;

public class RobotContainer {
  private final CommandSwerveDrive swerveDrive;
  private final TeleopControls teleopControls;
  private final DriveStraightAuto driveStraightAuto;
  private ShooterRoller shooterRoller;

  public RobotContainer() {
    LoggingUtil.logGitProperties();

    swerveDrive = new CommandSwerveDrive(LearnBotConstants.getDrivetrainConstants());

    teleopControls = new TeleopControls(this);
    teleopControls.configureBindings();

    driveStraightAuto = new DriveStraightAuto(this);
    shooterRoller = new ShooterRoller();
  }

  public CommandSwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Command getAutonomousCommand() {

    //return driveStraightAuto.getCommand();
    return shooterRoller.shoot(15);
  }

  public void latePeriodic() {
    swerveDrive.latePeriodic();
  }
}
