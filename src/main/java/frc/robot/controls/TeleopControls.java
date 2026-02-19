package frc.robot.controls;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Preferences;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoClimb;
import frc.robot.subsystems.hood.ShooterHoodConstants;
import frc.robot.subsystems.turret.TurretConstants;

public class TeleopControls {
  private RobotContainer robot;
  private AutoClimb autoClimb;
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  private boolean fineControl = false;

  public TeleopControls(RobotContainer robot) {
    this.robot = robot;
    this.autoClimb = new AutoClimb(robot);
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

    Command teleopSwerveCommand =
        new XBoxTeleopSwerveCommand(
            robot.getSwerveDrive(), Preferences.apply(new XBoxTeleopSwerveConstants()));

    teleopEnabledTrigger.whileTrue(teleopSwerveCommand);

    // Configure operator controls and automated driver controls
    // driver.a().onTrue(Commands.print("Nothing"));
    driver.b().onTrue(autoClimb.climb());
    driver.x().onTrue(autoClimb.unclimb());
    // driver.y().onTrue(Commands.print("Reset Heading"));
    driver.leftBumper().whileTrue(Commands.print("Depot"));
    // driver.leftTrigger().onTrue(Commands.print("Super Boost"));
    driver.rightBumper().whileTrue(Commands.print("Outpost"));
    // driver.rightTrigger().onTrue(Commands.print("Boost"));
    driver
        .leftStick()
        .whileTrue(
            Commands.parallel(
                this.robot.getIntakeRollers().outtake(), this.robot.getHopper().dump()));
    driver
        .rightStick()
        .whileTrue(
            this.robot
                .getDriveToClumpCommand()
                .driveToClump(
                    this.robot.getIntakeExtension(),
                    this.robot.getIntakeRollers(),
                    this.robot.getFuelLocalization(),
                    this.robot.getSwerveDrive()));
    driver
        .start()
        .whileTrue(this.robot.getIntakeRollers().intake()); // this might be switched with back

    operator.a().onTrue(robot.getClimb().descend()); // Lower climb
    operator.b().onTrue(robot.getClimb().pullUp()); // Lift robot
    operator.y().onTrue(robot.getClimb().elevate()); // Raise climb
    operator.leftBumper().whileTrue(robot.getHopper().unjam()); // Unjam hopper
    operator
        .leftTrigger()
        .whileTrue(robot.getShooterRollers().shoot(RotationsPerSecond.of(0))); // Disable shoot
    operator
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  fineControl = !fineControl;
                }));
    operator.rightTrigger().whileTrue(Commands.print("Force Shoot"));
    operator
        .leftStick()
        .whileTrue(
            Commands.parallel( // Dump fuel
                    robot.getHopper().getKicker().reverse(),
                    robot.getHopper().getBeltFloor().dump(),
                    robot.getIntakeRollers().outtake())
                .onlyIf(robot.getIntakeExtension()::isExtended));
    operator.back().whileTrue(Commands.print("Pass Left")); // this might be switched with start
    operator.start().whileTrue(Commands.print("Pass Right")); // this might be switched with back
    operator
        .povUp()
        .whileTrue(
            this.robot
                .getShooterHood()
                .moveAtVoltage(ShooterHoodConstants.FINE_CONTROL_VOLTAGE)); // CHECK SIGN
    operator
        .povDown()
        .whileTrue(
            this.robot
                .getShooterHood()
                .moveAtVoltage(
                    ShooterHoodConstants.FINE_CONTROL_VOLTAGE.unaryMinus())); // CHECK SIGN
    operator
        .povLeft()
        .whileTrue(
            this.robot
                .getTurret()
                .moveAtVoltage(TurretConstants.FINE_CONTROL_VOLTAGE)); // CHECK SIGN
    operator
        .povLeft()
        .whileTrue(
            this.robot
                .getTurret()
                .moveAtVoltage(TurretConstants.FINE_CONTROL_VOLTAGE.unaryMinus())); // CHECK SIGN

    // Intake extension and retraction
    Trigger intakeRetract = operator.rightStick().or(driver.back());
    Trigger intakeExtend = intakeRetract.negate().and(RobotState::isTeleop);

    intakeRetract.whileTrue(robot.getIntakeExtension().retract());
    intakeExtend.onTrue(robot.getIntakeExtension().extend());
  }
}
