// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.vision.AprilTagVision;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.DriveStraightAuto;
import frc.robot.controls.TeleopControls;
import frc.robot.learnbot.LearnBotConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.intakerollers.IntakeRollers;

public class RobotContainer {
  private final CommandSwerveDrive swerveDrive;
  private final TeleopControls teleopControls;
  private final DriveStraightAuto driveStraightAuto;
  private final IntakeRollers intakeRollers;
  private final AprilTagVision aprilTagVision;
  private final Climb climb;

  public RobotContainer() {
    LoggingUtil.logGitProperties();

    intakeRollers = new IntakeRollers();
    swerveDrive =
        new CommandSwerveDrive(Preferences.apply(LearnBotConstants.getDrivetrainConstants()));

    climb = new Climb();

    aprilTagVision =
        new AprilTagVision(swerveDrive, LearnBotConstants.getAprilTagVisionConstants());

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
    // return driveStraightAuto.getCommand();
    return climb.elevate();
  }

  public void latePeriodic() {
    swerveDrive.latePeriodic();
  }
}
