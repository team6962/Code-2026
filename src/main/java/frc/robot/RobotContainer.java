// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.team6962.lib.logging.CurrentDrawLogger;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.vision.AprilTagVision;
import com.team6962.lib.vision.SphereClumpLocalization;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoChooserOption;
import frc.robot.auto.AutoLowerHood;
import frc.robot.auto.Autonomous;
import frc.robot.auto.DriveStraightAuto;
import frc.robot.auto.shoot.ShooterFunctions;
import frc.robot.constants.RobotConstants;
import frc.robot.controls.TeleopControls;
import frc.robot.subsystems.hood.ShooterHood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intakeextension.IntakeExtension;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.subsystems.shooterrollers.ShooterRollers;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.visualizer.RobotVisualizer;

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
  // private final Climb climb;
  private final Hopper hopper;
  private final RobotVisualizer visualizer;
  private final SendableChooser<AutoChooserOption> autoChooser = new SendableChooser<>();
  private final ShooterFunctions hubFunctions;
  private final ShooterFunctions passFunctions;
  private final Autonomous autonomous;
  private final Command noneAutonomous = Commands.none();

  public RobotContainer() {
    DogLog.setOptions(new DogLogOptions().withNtPublish(RobotBase::isSimulation));

    LoggingUtil.logGitProperties();
    RobotController.setBrownoutVoltage(4.6);

    CurrentDrawLogger.start();

    constants = RobotConstants.generate();

    swerveDrive = new CommandSwerveDrive(constants.getDrivetrainConstants());

    // climb = new Climb();
    AutoLowerHood autoLowerHood = new AutoLowerHood(swerveDrive);
    shooterHood = new ShooterHood(autoLowerHood::shouldLowerHood);
    intakeRollers = new IntakeRollers();
    shooterRollers = new ShooterRollers();
    turret = new Turret();
    intakeExtension = new IntakeExtension();
    hopper = new Hopper();

    aprilTagVision = new AprilTagVision(swerveDrive, constants.getAprilTagVisionConstants());
    fuelClumpLocalization =
        new SphereClumpLocalization(swerveDrive, constants.getSphereCameraConstants());
    hubFunctions =
        new ShooterFunctions(
            RobotBase.isSimulation() ? "sim_shooter_hub_data.csv" : "shooter_hub_data.csv");
    passFunctions = new ShooterFunctions("shooter_pass_data.csv");
    teleopControls = new TeleopControls(this);
    teleopControls.configureBindings();

    driveStraightAuto = new DriveStraightAuto(this);
    autonomous = new Autonomous(this);

    configureAutonomousChooser();

    visualizer = new RobotVisualizer(this);
  }

  private void configureAutonomousChooser() {
    // Set "Do Nothing" as the default option
    autoChooser.setDefaultOption("Do Nothing", new AutoChooserOption(noneAutonomous, false));

    if (RobotBase.isSimulation()) {
      autoChooser.addOption(
          "Test Drive To Pose",
          new AutoChooserOption(
              swerveDrive
                  .driveTo(
                      new Pose2d(3, 4.03463125, Rotation2d.fromDegrees(0)),
                      new ChassisSpeeds(0, 0, 0))
                  .repeatedly(),
              false));

      autoChooser.addOption(
          "Test Drive To Pose with Final Velocity",
          new AutoChooserOption(
              swerveDrive
                  .driveTo(
                      new Pose2d(10, 5, Rotation2d.fromDegrees(0)), new ChassisSpeeds(-2, 2, 0))
                  .andThen(swerveDrive.drive(new ChassisSpeeds(-2, 2, 0))),
              false));
    }

    autoChooser.addOption(
        "Left Single Neutral Cycle",
        new AutoChooserOption(autonomous.leftSingleNeutralCycle(), true));
    autoChooser.addOption(
        "Left Double Neutral Cycle",
        new AutoChooserOption(autonomous.leftDoubleNeutralCycle(), true));
    autoChooser.addOption(
        "Right Single Neutral Cycle",
        new AutoChooserOption(autonomous.rightSingleNeutralCycle(), true));
    autoChooser.addOption(
        "Right Double Neutral Cycle",
        new AutoChooserOption(autonomous.rightDoubleNeutralCycle(), true));

    autoChooser.addOption("Shoot Preload", new AutoChooserOption(autonomous.preload(), true));
    autoChooser.addOption(
        "Back Up and Shoot Preload",
        new AutoChooserOption(autonomous.moveBackwardAndShoot(), true));

    autoChooser.addOption(
        "Drive Straight", new AutoChooserOption(driveStraightAuto.getCommand(), true));

    autoChooser.addOption(
        "SysId Shooter Rollers", new AutoChooserOption(shooterRollers.sysId(), false));

    autoChooser.addOption(
        "SysId Front Left Steer",
        new AutoChooserOption(swerveDrive.getModules()[0].getSteerMechanism().sysId(), false));
    autoChooser.addOption(
        "SysId Front Right Steer",
        new AutoChooserOption(swerveDrive.getModules()[1].getSteerMechanism().sysId(), false));
    autoChooser.addOption(
        "SysId Back Left Steer",
        new AutoChooserOption(swerveDrive.getModules()[2].getSteerMechanism().sysId(), false));
    autoChooser.addOption(
        "SysId Back Right Steer",
        new AutoChooserOption(swerveDrive.getModules()[3].getSteerMechanism().sysId(), false));
    autoChooser.addOption(
        "SysId Front Drive",
        new AutoChooserOption(
            swerveDrive.driveSysId("Front Drive", true, true, false, false, 0), false));
    autoChooser.addOption(
        "SysId Back Drive",
        new AutoChooserOption(
            swerveDrive.driveSysId("Back Drive", false, false, true, true, 2), false));
    autoChooser.addOption(
        "Calibrate Wheel Size", new AutoChooserOption(swerveDrive.calibrateWheelSize(), false));

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
    return autoChooser.getSelected() != null ? autoChooser.getSelected().command : Commands.none();
  }

  public void periodic() {
    DogLog.forceNt.log("BatteryVoltage", RobotController.getBatteryVoltage());
    DogLog.forceNt.log("FMSConnected", DriverStation.isFMSAttached());
    DogLog.forceNt.log("VoltageHigh", RobotController.getBatteryVoltage() > 12.5);

    if (RobotState.isDisabled()) {
      DogLog.forceNt.log(
          "Auto Routine Selected",
          autoChooser.getSelected() != null && autoChooser.getSelected().recommended);
    }
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

  // public Climb getClimb() {
  //   return climb;
  // }

  public Hopper getHopper() {
    return hopper;
  }

  public ShooterFunctions getHubFunctions() {
    return hubFunctions;
  }

  public ShooterFunctions getPassFunctions() {
    return passFunctions;
  }

  public RobotVisualizer getVisualizer() {
    return visualizer;
  }
}
