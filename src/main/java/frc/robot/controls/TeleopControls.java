package frc.robot.controls;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.auto.TrenchDriving;
import frc.robot.auto.shoot.AutoShoot;
import frc.robot.subsystems.hood.ShooterHoodConstants;
import frc.robot.subsystems.intakeextension.IntakeExtensionConstants;
import frc.robot.subsystems.shooterrollers.ShooterRollersConstants;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.List;
import java.util.Set;

public class TeleopControls extends SubsystemBase {
  private RobotContainer robot;
  // private AutoClimb autoClimb;
  //   private ShootFuel shootFuel;
  //   private AutoOutpost autoOutpost;
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);
  private Distance shootingTestDistance = Inches.of(206);
  //   private AutoDepot autoDepot;
  //   private AutoZoneDefense autoZoneDefense = new AutoZoneDefense(robot);
  private AutoShoot autoShoot;

  private ControllerRumble driverRumble = new ControllerRumble(driver);
  private ControllerRumble operatorRumble = new ControllerRumble(operator);

  private static boolean fineControl = false;
  private AngularVelocity flywheelVelocity = ShooterRollersConstants.FIXED_FLYWHEEL_VELOCITY;
  private double tunableHoodAngle = 0;
  private double tunableRollerVelocity = 0;

  private double hubMaxLinearVelocity = 1.5;
  private double hubMaxAngularVelocity = 0.25;
  private double hubMaxLinearAcceleration = 2;
  private double hubMaxAngularAcceleration = 0.25;
  private double passMaxLinearVelocity = 2;
  private double passMaxAngularVelocity = 0.5;
  private double agitationWaitTime = 3.0;
  private double agitationOnTime = 0.5;

  public static boolean isFineControlOn() {
    return fineControl;
  }

  public TeleopControls(RobotContainer robot) {
    this.robot = robot;

    DogLog.tunable(
        "TeleopControls/hubMaxLinearVelocity",
        hubMaxLinearVelocity,
        value -> hubMaxLinearVelocity = value);
    DogLog.tunable(
        "TeleopControls/hubMaxAngularVelocity",
        hubMaxAngularVelocity,
        value -> hubMaxAngularVelocity = value);
    DogLog.tunable(
        "TeleopControls/hubMaxLinearAcceleration",
        hubMaxLinearAcceleration,
        value -> hubMaxLinearAcceleration = value);
    DogLog.tunable(
        "TeleopControls/hubMaxAngularAcceleration",
        hubMaxAngularAcceleration,
        value -> hubMaxAngularAcceleration = value);
    DogLog.tunable(
        "TeleopControls/passMaxLinearVelocity",
        passMaxLinearVelocity,
        value -> passMaxLinearVelocity = value);
    DogLog.tunable(
        "TeleopControls/passMaxAngularVelocity",
        passMaxAngularVelocity,
        value -> passMaxAngularVelocity = value);

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

    DogLog.tunable(
        "TeleopControls/Agitation Wait Time",
        agitationWaitTime,
        value -> {
          agitationWaitTime = value;
        });

    DogLog.tunable(
        "TeleopControls/Agitation On Time",
        agitationOnTime,
        value -> {
          agitationOnTime = value;
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

    // Dump fuel
    driver
        .leftStick()
        .whileTrue(
            Commands.parallel(
                this.robot.getIntakeRollers().outtake(), this.robot.getHopper().dump()));

    // Set up triggers for shooting and intake controls
    Trigger shootButtonsTrigger =
        operator.rightTrigger().or(driver.back()).and(RobotState::isEnabled);

    Trigger intakeTrigger = driver.rightStick().and(RobotState::isEnabled);

    // Intakes fuel, agitates roller floor when not shooting

    intakeTrigger.whileTrue(
        this.robot
            .getIntakeRollers()
            .intake()
            .alongWith(robot.getIntakeExtension().requestExtend()));

    intakeTrigger
        .and(shootButtonsTrigger.negate())
        .whileTrue(
            Commands.sequence(
                robot.getHopper().getFloor().agitate().withTimeout(agitationOnTime),
                Commands.waitSeconds(agitationWaitTime)));

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

    // Unjam hopper
    operator.leftBumper().or(driver.leftBumper()).whileTrue(robot.getHopper().unjam());

    // Backup shoot
    operator.leftTrigger().and(RobotState::isEnabled).whileTrue(robot.getHopper().feed());

    // Toggle fine control mode
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

    // Stop Shooting
    operator.x().whileTrue(robot.getShooterRollers().shoot(RotationsPerSecond.of(0)));

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
    SmartDashboard.putData("Zero Shooter Hood", this.robot.getShooterHood().zero());
    SmartDashboard.putData("Zero Turret", robot.getTurret().zero());
    SmartDashboard.putData("Zero Intake Retracted", robot.getIntakeExtension().zeroRetracted());
    SmartDashboard.putData("Zero Intake Extended", robot.getIntakeExtension().zeroExtended());

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

    operator
        .povLeft()
        .and(() -> fineControl)
        .whileTrue(this.robot.getTurret().moveAtVoltage(TurretConstants.FINE_CONTROL_VOLTAGE));

    operator
        .povRight()
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

    autoShoot =
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
            .and(operator.x().negate())
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
            .and(operator.leftTrigger().negate())
            .and(operator.x().negate())
            .and(() -> !fineControl);

    autoPassTrigger.whileTrue(autoPass);

    shootButtonsTrigger
        .and(inAllianceZone)
        .whileTrue(
            Commands.defer(
                () ->
                    teleopSwerveCommand
                        .limitVelocity(
                            MetersPerSecond.of(hubMaxLinearVelocity),
                            RotationsPerSecond.of(hubMaxAngularVelocity))
                        .alongWith(
                            teleopSwerveCommand.limitAcceleration(
                                MetersPerSecondPerSecond.of(hubMaxLinearAcceleration),
                                RotationsPerSecondPerSecond.of(hubMaxAngularAcceleration))),
                Set.of()))
        .and(autoShoot.isReadyToShoot())
        .whileTrue(robot.getHopper().feed())
        .and(driver.rightStick().negate())
        .whileTrue(
            robot.getIntakeExtension().agitate().alongWith(robot.getIntakeRollers().intakeSlow()));

    shootButtonsTrigger
        .and(inAllianceZone.negate())
        .whileTrue(
            Commands.defer(
                () ->
                    teleopSwerveCommand.limitVelocity(
                        MetersPerSecond.of(passMaxLinearVelocity),
                        RotationsPerSecond.of(passMaxAngularVelocity)),
                Set.of()))
        .and(autoPass.isReadyToShoot())
        .whileTrue(robot.getHopper().feed());

    // Automatically load fuel into the kicker when there is fuel in the hopper

    Trigger load =
        new Trigger(() -> RobotState.isTeleop() && RobotState.isEnabled())
            .and(() -> !fineControl)
            .and(shootButtonsTrigger.negate())
            .and(operator.x().negate())
            .and(operator.leftTrigger().negate())
            .and(() -> !robot.getHopper().getSensors().isKickerFull())
            .and(() -> !robot.getHopper().isEmpty())
            .and(intakeTrigger.negate());

    load.whileTrue(robot.getHopper().load());

    // Intake extension and retraction
    Trigger intakeRetract = operator.rightStick().or(driver.start());
    Trigger intakeExtend =
        intakeRetract
            .negate()
            .and(RobotState::isTeleop)
            .and(RobotState::isEnabled)
            .and(shootButtonsTrigger.and(inAllianceZone).and(autoShoot.isReadyToShoot()).negate());

    intakeRetract.and(() -> !fineControl).whileTrue(robot.getIntakeExtension().retract());
    intakeExtend.and(() -> !fineControl).whileTrue(robot.getIntakeExtension().extend());

    new ShiftFeedback(List.of(driverRumble, operatorRumble));
  }

  private Command fineControlEnableRumble() {
    return Commands.sequence(
        operatorRumble.rumble(1, 0).withTimeout(1.0 / 3.0),
        Commands.waitSeconds(1.0 / 3.0),
        operatorRumble.rumble(0, 1).withTimeout(1.0 / 3.0));
  }

  private Command fineControlDisableRumble() {
    return operatorRumble.rumble(1, 1).withTimeout(1.0);
  }

  @Override
  public void periodic() {
    if (!fineControl) {
      autoShoot.setHoodOffset(mapOffset(operator.getLeftY(), Degrees.of(5)));
      autoShoot.setTurretOffset(mapOffset(-operator.getLeftX(), Degrees.of(10)));
      autoShoot.setFlywheelSpeedOffset(mapOffset(-operator.getRightY(), RotationsPerSecond.of(2)));
    } else {
      autoShoot.setHoodOffset(Degrees.of(0));
      autoShoot.setTurretOffset(Degrees.of(0));
      autoShoot.setFlywheelSpeedOffset(RotationsPerSecond.of(0));
    }

    ControllerLogging.logInputs(driver.getHID());
    ControllerLogging.logInputs(operator.getHID());

    DogLog.forceNt.log("TeleopControls/DriverConnected", driver.isConnected());
    DogLog.forceNt.log("TeleopControls/OperatorConnected", operator.isConnected());
  }

  private Angle mapOffset(double joystickInput, Angle maxOffset) {
    if (Math.abs(joystickInput) < 0.1) {
      return Degrees.of(0);
    } else {
      joystickInput -= Math.signum(joystickInput) * 0.1;
      joystickInput /= 0.9;

      return Degrees.of(
          Math.signum(joystickInput) * (joystickInput * joystickInput) * maxOffset.in(Degrees));
    }
  }

  private AngularVelocity mapOffset(double joystickInput, AngularVelocity maxOffset) {
    if (Math.abs(joystickInput) < 0.1) {
      return RotationsPerSecond.of(0);
    } else {
      joystickInput -= Math.signum(joystickInput) * 0.1;
      joystickInput /= 0.9;

      return RotationsPerSecond.of(
          Math.signum(joystickInput)
              * (joystickInput * joystickInput)
              * maxOffset.in(RotationsPerSecond));
    }
  }
}
