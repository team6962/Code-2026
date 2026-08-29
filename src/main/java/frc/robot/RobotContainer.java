// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoChooserOption;
import frc.robot.auto.AutoLowerHood;
import frc.robot.auto.Autonomous;
import frc.robot.auto.DriveStraightAuto;
import frc.robot.auto.TrenchDriving;
import frc.robot.auto.shoot.AutoShoot;
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

  private boolean autoShoot = false;
  private boolean autoIntake = false;

  private Pose2d startPose;

  public RobotContainer() {
    DogLog.setOptions(new DogLogOptions().withNtPublish(RobotBase::isSimulation));

    LoggingUtil.logGitProperties();
    RobotController.setBrownoutVoltage(5.4);

    CurrentDrawLogger.start();

    constants = RobotConstants.generate();

    intakeExtension = new IntakeExtension();
    NamedCommands.registerCommand("startIntake", Commands.runOnce(() -> autoIntake = true));
    NamedCommands.registerCommand("stopIntake", Commands.runOnce(() -> autoIntake = false));
    NamedCommands.registerCommand("startShoot", Commands.runOnce(() -> autoShoot = true));
    NamedCommands.registerCommand("stopShoot", Commands.runOnce(() -> autoShoot = false));
    NamedCommands.registerCommand("extendIntake", intakeExtension.extend());
    NamedCommands.registerCommand("retractIntake", intakeExtension.retract());

    swerveDrive = new CommandSwerveDrive(constants.getDrivetrainConstants());

    // climb = new Climb();
    AutoLowerHood autoLowerHood = new AutoLowerHood(swerveDrive);
    shooterHood = new ShooterHood(autoLowerHood::shouldLowerHood);
    intakeRollers = new IntakeRollers();
    shooterRollers = new ShooterRollers();
    turret = new Turret();
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

    // Warm up the path following pipeline
    CommandScheduler.getInstance()
        .schedule(
            swerveDrive
                .followPath("center.0")
                .withTimeout(3)
                .until(RobotState::isEnabled)
                .ignoringDisable(true));

    Trigger intakeTrigger = new Trigger(() -> autoIntake);
    intakeTrigger.whileTrue(intakeRollers.intake());

    Trigger inAllianceZone =
        new Trigger(
            () ->
                swerveDrive.getPosition2d().getX()
                    < TrenchDriving.OBSTACLES_CENTER_X.in(Meters));
    Trigger aimTrigger = new Trigger(() -> autoShoot);
    AutoShoot autoShootCommand = new AutoShoot(
            swerveDrive,
            turret,
            shooterHood,
            shooterRollers,
            hubFunctions,
            () -> AutoShoot.HUB_TRANSLATION,
            () -> null,
            () -> null);
    aimTrigger.whileTrue(autoShootCommand);
    Trigger shootTrigger = aimTrigger.and(inAllianceZone).and(autoShootCommand.isReadyToShoot());
    shootTrigger.whileTrue(hopper.feed());
    
    PathPlannerPath path = swerveDrive.loadChoreoPath("auto");

    if (path.getPathPoses().size() == 0) {
      throw new RuntimeException("Path has no poses defined.");
    }

    startPose = path.getPathPoses().get(0);

    swerveDrive.getLocalization().resetPosition(startPose);
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
        new AutoChooserOption(autonomous.leftSingleNeutralCycle(), false));
    autoChooser.addOption(
        "Left Double Neutral Cycle",
        new AutoChooserOption(autonomous.leftDoubleNeutralCycle(), false));
    autoChooser.addOption(
        "Greedy Left Double Sweep Bump",
        new AutoChooserOption(autonomous.greedyLeftDoubleSweepBump(), true));
    autoChooser.addOption(
        "Right Single Neutral Cycle",
        new AutoChooserOption(autonomous.rightSingleNeutralCycle(), false));
    autoChooser.addOption(
        "Right Double Neutral Cycle",
        new AutoChooserOption(autonomous.rightDoubleNeutralCycle(), false));
    autoChooser.addOption(
        "Greedy Right Double Sweep Bump",
        new AutoChooserOption(autonomous.greedyRightDoubleSweepBump(), true));
    autoChooser.addOption(
        "Safe Right Double Sweep Bump",
        new AutoChooserOption(autonomous.safeRightDoubleSweepBump(), true));
    autoChooser.addOption(
        "Safe Left Double Sweep Bump",
        new AutoChooserOption(autonomous.safeLeftDoubleSweepBump(), true));

    autoChooser.addOption("Shoot Preload", new AutoChooserOption(autonomous.preload(), true));
    autoChooser.addOption(
        "Back Up and Shoot Preload",
        new AutoChooserOption(autonomous.moveBackwardAndShoot(), true));

    autoChooser.addOption(
        "Center from Bump", new AutoChooserOption(autonomous.center(false), true));
    autoChooser.addOption("Center from Hub", new AutoChooserOption(autonomous.center(true), true));

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
    // return autoChooser.getSelected() != null ? autoChooser.getSelected().command : Commands.none();

    return Commands.sequence(
      Commands.runOnce(() -> swerveDrive.getLocalization().resetPosition(startPose)),
      swerveDrive.followPath("auto")
    );
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
