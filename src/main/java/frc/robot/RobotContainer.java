// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.swerve.CommandSwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.DriveStraightAuto;
import frc.robot.controls.TeleopControls;
import frc.robot.learnbot.LearnBotConstants;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  private final CommandSwerveDrive swerveDrive;
  private final TeleopControls teleopControls;
  private final DriveStraightAuto driveStraightAuto;

  private IntakeRollers intakeRoller;
  private Turret turret;

  public RobotContainer() {
    LoggingUtil.logGitProperties();

    swerveDrive = new CommandSwerveDrive(LearnBotConstants.getDrivetrainConstants());

    teleopControls = new TeleopControls(this);
    teleopControls.configureBindings();

    driveStraightAuto = new DriveStraightAuto(this);
    intakeRoller = new IntakeRollers();
    turret = new Turret();
  }

  public CommandSwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public Command getAutonomousCommand() {
    // return driveStraightAuto.getCommand();
    // return Commands.sequence(intakeRoller.move().withTimeout(1),Commands.waitSeconds(0.5)).repeatedly();
    return turret.moveTo(Rotations.of(1));
  }

  public void latePeriodic() {
    // swerveDrive.latePeriodic();
  }
}
