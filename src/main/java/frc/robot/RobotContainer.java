// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.swerve.CommandSwerveDrive;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.DriveStraightAuto;
import frc.robot.controls.TeleopControls;
import frc.robot.learnbot.LearnBotConstants;
import frc.robot.subsystems.turret.TurretRotation;

public class RobotContainer {
  private final CommandSwerveDrive swerveDrive;
  private final TeleopControls teleopControls;
  private final TurretRotation turretRotation = new TurretRotation();
  double angleInRadians = Math.PI / 2;

  public RobotContainer() {
    LoggingUtil.logGitProperties();

    swerveDrive = new CommandSwerveDrive(LearnBotConstants.getDrivetrainConstants());

    teleopControls = new TeleopControls(this);
    teleopControls.configureBindings();
  }

  public CommandSwerveDrive getSwerveDrive() {
   return swerveDrive;
  }

  public Command getAutonomousCommand() {
    return turretRotation.moveTo(Radians.of(3));
  }

  public void latePeriodic() {
    // swerveDrive.latePeriodic();
  }
}
