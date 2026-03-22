package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
// import frc.robot.auto.AutoClimb;
import frc.robot.auto.AutoDepot;
import frc.robot.auto.AutoOutpost;
import frc.robot.auto.ShootFuel;
import frc.robot.auto.TrenchDriving;
import frc.robot.auto.shoot.AutoShoot;
import frc.robot.subsystems.hood.ShooterHoodConstants;
import frc.robot.subsystems.intakeextension.IntakeExtensionConstants;
import frc.robot.subsystems.shooterrollers.ShooterRollersConstants;
import frc.robot.subsystems.turret.TurretConstants;

public class TeleopControls {
  private RobotContainer robot;
  // private AutoClimb autoClimb;
  private ShootFuel shootFuel;
  private AutoOutpost autoOutpost;
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);
  private Distance shootingTestDistance = Inches.of(206);
  private AutoDepot autoDepot;

  private boolean fineControl = false;
  private AngularVelocity flywheelVelocity = ShooterRollersConstants.FIXED_FLYWHEEL_VELOCITY;
  private double tunableHoodAngle = 0;
  private double tunableRollerVelocity = 0;

  public TeleopControls(RobotContainer robot) {
    this.robot = robot;
    // this.autoClimb = new AutoClimb(robot);
    this.shootFuel = new ShootFuel(robot);
    this.autoOutpost = new AutoOutpost(robot, shootFuel);
    this.autoDepot = new AutoDepot(robot);

    DogLog.forceNt.log(
        "TeleopControls/FineControl", fineControl); // Initial log so that the folder shows up

    DogLog.tunable(
        "FlywheelVelocity",
        flywheelVelocity.in(RotationsPerSecond),
        value -> {
          flywheelVelocity = RotationsPerSecond.of(value);
        });

    DogLog.tunable(
        "Shooting Test Distance (in)",
        shootingTestDistance.in(Inches),
        value -> {
          shootingTestDistance = Inches.of(value);
        });

    DogLog.tunable(
        "Override Shooting/Hood Angle",
        tunableHoodAngle,
        value -> {
          tunableHoodAngle = value;
        });

    DogLog.tunable(
        "Override Shooting/Roller Velocity",
        tunableRollerVelocity,
        value -> {
          tunableRollerVelocity = value;
        });
  }

  public void configureBindings() {
    // Silence joystick connection warnings in simulation because they are
    // annoying and not useful
    if (RobotBase.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Configure basic driver controls
    Trigger teleopEnabledTrigger =
        new Trigger(() -> RobotState.isTeleop() && RobotState.isEnabled());

    XBoxTeleopSwerveCommand teleopSwerveCommand =
        new XBoxTeleopSwerveCommand(
            robot.getSwerveDrive(), robot.getConstants().getTeleopSwerveConstants());

    teleopEnabledTrigger.whileTrue(teleopSwerveCommand);

    // Configure operator controls and automated driver controls

    // Driver A is unused
    // Driver Y resets heading (configured by XBoxTeleopSwerveCommand)
    // Driver right trigger is boost (configured by XBoxTeleopSwerveCommand)
    // Driver left trigger is super boost (configured by XBoxTeleopSwerveCommand)

    // Auto Climb and Unclimb
    // driver.b().onTrue(autoClimb.climb());
    // driver.x().onTrue(autoClimb.unclimb());

    // Auto Depot
    driver.leftBumper().whileTrue(autoDepot.autoDepot());

    driver // Auto Drive to Outpost
        .rightBumper()
        .whileTrue(autoOutpost.autoOutpost());

    // Dump fuel - WORKS
    driver
        .leftStick()
        .whileTrue(
            Commands.parallel(
                this.robot.getIntakeRollers().outtake(), this.robot.getHopper().dump()));

    // Intake and drive to fuel clump
    // driver.start().whileTrue(driveToClump.driveToClump());

    // Intake without driving - WORKS
    driver
        .rightStick()
        .whileTrue(this.robot.getIntakeRollers().intake()); // this might be switched with back

    // Manual climb controls
    // operator.a().onTrue(robot.getClimb().descend()); // Lower climb
    // operator.b().onTrue(robot.getClimb().pullUp()); // Lift robot
    // operator.y().onTrue(robot.getClimb().elevate()); // Raise climb

    operator
        .b()
        .and(RobotState::isDisabled)
        .onTrue(Commands.runOnce(() -> robot.getTurret().zero()).ignoringDisable(true));

    // Manual lower hood
    operator.y().whileTrue(robot.getShooterHood().moveTo(ShooterHoodConstants.MIN_ANGLE));

    // Fixed backup shoot
    operator
        .a()
        .whileTrue(
            Commands.parallel(
                robot.getShooterHood().moveTo(Degrees.of(22.5)),
                robot.getTurret().moveTo(Degrees.of(180)),
                robot.getShooterRollers().shoot(RotationsPerSecond.of(22.5))));

    // Unjam hopper - WORKS
    operator.leftBumper().whileTrue(robot.getHopper().unjam());

    // Disable shooting
    operator.leftTrigger().whileTrue(robot.getShooterRollers().shoot(RotationsPerSecond.of(0)));

    // Toggle fine control mode - WORKS
    operator
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      fineControl = !fineControl;
                      DogLog.forceNt.log("TeleopControls/FineControl", fineControl);
                    })
                .andThen(
                    Commands.either(
                        fineControlEnableRumble(), fineControlDisableRumble(), () -> fineControl))
                .ignoringDisable(true));

    // Shooting
    operator.x().whileTrue(robot.getHopper().feed());

    // Fine control
    operator
        .povUp()
        .and(() -> fineControl)
        .whileTrue(
            this.robot.getShooterHood().moveAtVoltage(ShooterHoodConstants.FINE_CONTROL_VOLTAGE));

    operator
        .povDown()
        .and(() -> fineControl)
        .whileTrue(
            this.robot
                .getShooterHood()
                .moveAtVoltage(ShooterHoodConstants.FINE_CONTROL_VOLTAGE.unaryMinus()));

    // Backup zero
    SmartDashboard.putData(
        "Shooter Hood Zeroing", this.robot.getShooterHood().zero().onlyIf(RobotState::isDisabled));

    operator
        .povLeft()
        .and(() -> fineControl)
        .whileTrue(this.robot.getTurret().moveAtVoltage(TurretConstants.FINE_CONTROL_VOLTAGE));

    operator
        .povLeft()
        .and(() -> fineControl)
        .whileTrue(
            this.robot
                .getTurret()
                .moveAtVoltage(TurretConstants.FINE_CONTROL_VOLTAGE.unaryMinus()));

    operator // WORKS
        .axisGreaterThan(Axis.kLeftX.value, 0.5)
        .and(() -> fineControl)
        .whileTrue(
            DogLog.time(
                "fineControlOut",
                this.robot
                    .getIntakeExtension()
                    .moveAtVoltage(IntakeExtensionConstants.FINE_CONTROL_VOLTAGE)));

    operator // WORKS
        .axisLessThan(Axis.kLeftX.value, -0.5)
        .and(() -> fineControl)
        .whileTrue(
            DogLog.time(
                "fineControlIn",
                this.robot
                    .getIntakeExtension()
                    .moveAtVoltage(IntakeExtensionConstants.FINE_CONTROL_VOLTAGE.unaryMinus())));

    // operator
    //     .axisGreaterThan(Axis.kRightY.value, 0.5)
    //     .and(() -> fineControl)
    //     .whileTrue(this.robot.getClimb().moveAtVoltage(ClimbConstants.FINE_CONTROL_VOLTAGE));

    // operator
    //     .axisLessThan(Axis.kRightY.value, -0.5)
    //     .and(() -> fineControl)
    //     .whileTrue(
    //
    // this.robot.getClimb().moveAtVoltage(ClimbConstants.FINE_CONTROL_VOLTAGE.unaryMinus()));

    // Intake extension and retraction - WORKS
    Trigger intakeRetract = operator.rightStick();
    Trigger intakeExtend =
        intakeRetract.negate().and(RobotState::isTeleop).and(RobotState::isEnabled);

    intakeRetract.and(() -> !fineControl).whileTrue(robot.getIntakeExtension().retract());
    intakeExtend.and(() -> !fineControl).whileTrue(robot.getIntakeExtension().extend());

    // Automatically load fuel into the kicker when there is fuel in the hopper - WORKS, but not
    // fully tested
    Trigger load =
        new Trigger(() -> RobotState.isTeleop() && RobotState.isEnabled())
            .and(() -> !fineControl)
            .and(driver.leftStick().negate())
            .and(operator.leftBumper().negate())
            .and(operator.rightTrigger().negate())
            .and(driver.a().negate())
            .and(operator.x().negate())
            .and(() -> !robot.getHopper().getSensors().isKickerFull())
            .and(() -> !robot.getHopper().isEmpty());

    load.whileTrue(robot.getHopper().load());

    // Climb retraction
    // Command autodescend = robot.getClimb().descend();
    // Trigger climbRetract =
    //     new Trigger(() -> CommandUtil.isClearToOverride(robot.getClimb(), autodescend))
    //         .and(RobotState::isTeleop)
    //         .and(RobotState::isEnabled);

    // climbRetract.onTrue(robot.getClimb().descend());

    AutoShoot autoShoot =
        new AutoShoot(
            robot.getSwerveDrive(),
            robot.getTurret(),
            robot.getShooterHood(),
            robot.getShooterRollers(),
            robot.getHubFunctions(),
            () -> AutoShoot.HUB_TRANSLATION,
            () -> tunableHoodAngle == 0 ? null : Degrees.of(tunableHoodAngle),
            () -> tunableRollerVelocity == 0 ? null : RotationsPerSecond.of(tunableRollerVelocity));

    Trigger inAllianceZone =
        new Trigger(
            () ->
                robot.getSwerveDrive().getPosition2d().getX()
                    < TrenchDriving.OBSTACLES_CENTER_X.in(Meters));

    Trigger autoshootTrigger =
        new Trigger(RobotState::isTeleop)
            .and(RobotState::isEnabled)
            .and(inAllianceZone)
            .and(operator.a().negate())
            .and(operator.y().negate())
            .and(() -> !fineControl);

    autoshootTrigger.whileTrue(autoShoot);

    AutoShoot autoPass =
        new AutoShoot(
            robot.getSwerveDrive(),
            robot.getTurret(),
            robot.getShooterHood(),
            robot.getShooterRollers(),
            robot.getPassFunctions(),
            () ->
                robot.getSwerveDrive().getPosition2d().getY() > AutoShoot.HUB_TRANSLATION.getY()
                    ? AutoShoot.PASS_RIGHT_TRANSLATION
                    : AutoShoot.PASS_LEFT_TRANSLATION,
            () -> tunableHoodAngle == 0 ? null : Degrees.of(tunableHoodAngle),
            () -> tunableRollerVelocity == 0 ? null : RotationsPerSecond.of(tunableRollerVelocity));

    Trigger autoPassTrigger =
        new Trigger(RobotState::isTeleop)
            .and(RobotState::isEnabled)
            .and(inAllianceZone.negate())
            .and(operator.a().negate())
            .and(operator.y().negate())
            .and(() -> !fineControl);

    autoPassTrigger.whileTrue(autoPass);

    Trigger shootButtonsTrigger = operator.rightTrigger().or(driver.back());

    shootButtonsTrigger
        .and(inAllianceZone)
        .whileTrue(
            teleopSwerveCommand.limitVelocity(
                MetersPerSecond.of(0.5), RotationsPerSecond.of(0.125))) // Temporary values
        .and(autoShoot.isReadyToShoot())
        .whileTrue(robot.getHopper().feed());

    shootButtonsTrigger
        .and(inAllianceZone.negate())
        .whileTrue(
            teleopSwerveCommand.limitVelocity(
                MetersPerSecond.of(1.5), RotationsPerSecond.of(0.5))) // Temporary values
        .and(autoPass.isReadyToShoot())
        .whileTrue(robot.getHopper().feed());

    // ShooterFunctions functions = robot.getHubFunctions();

    // driver
    //     .a()
    //     .whileTrue(
    //         Commands.parallel(
    //             robot
    //                 .getShooterRollers()
    //                 .shoot(() -> functions.getFlywheelVelocity(shootingTestDistance)),
    //             robot.getShooterHood().moveTo(() ->
    // functions.getHoodAngle(shootingTestDistance)),
    //             Commands.sequence(
    //                 Commands.waitUntil(
    //                     () ->
    //                         robot
    //                                 .getShooterRollers()
    //                                 .getAngularVelocity()
    //                                 .isNear(
    //                                     functions.getFlywheelVelocity(shootingTestDistance),
    //                                     RotationsPerSecond.of(1))
    //                             && robot
    //                                 .getShooterHood()
    //                                 .getPosition()
    //                                 .isNear(
    //                                     functions.getHoodAngle(shootingTestDistance),
    //                                     Degrees.of(1))),
    //                 robot.getHopper().feed())));
  }

  private Command rumble(
      CommandXboxController controller, RumbleType rumbleType, double intensity) {
    return Commands.startEnd(
        () -> controller.setRumble(rumbleType, intensity),
        () -> controller.setRumble(rumbleType, 0.0));
  }

  private Command fineControlEnableRumble() {
    return Commands.sequence(
        rumble(operator, RumbleType.kLeftRumble, 1).withTimeout(1.0 / 3.0),
        Commands.waitSeconds(1.0 / 3.0),
        rumble(operator, RumbleType.kRightRumble, 1).withTimeout(1.0 / 3.0));
  }

  private Command fineControlDisableRumble() {
    return Commands.sequence(rumble(operator, RumbleType.kBothRumble, 1).withTimeout(1.0));
  }
}
