package frc.robot.controls;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;
import com.team6962.lib.swerve.config.XBoxTeleopSwerveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.auto.DriveToClump;
import frc.robot.subsystems.hood.ShooterHoodConstants;
import frc.robot.subsystems.turret.TurretConstants;

public class TeleopControls {
  private RobotContainer robot;
  private AutoClimb autoClimb;
  private DriveToClump driveToClump;
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

    // Driver A is unused
    // Driver Y resets heading (configured by XBoxTeleopSwerveCommand)
    // Driver right trigger is boost (configured by XBoxTeleopSwerveCommand)
    // Driver left trigger is super boost (configured by XBoxTeleopSwerveCommand)

    // Auto Climb and Unclimb
    driver.b().onTrue(autoClimb.climb());
    driver.x().onTrue(autoClimb.unclimb());

    // Auto Depot
    driver
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                this.robot
                    .getSwerveDrive()
                    .driveTo(
                        new Pose2d(
                            1.518,
                            5.947,
                            new Rotation2d(
                                Radians.of(
                                    Math.PI)))), // rough position estimate based on simulation, not
                // exact
                this.robot.getIntakeExtension().extend(),
                Commands.parallel(
                    this.robot
                        .getSwerveDrive()
                        .driveTo(
                            new Pose2d(
                                0.546,
                                5.947,
                                new Rotation2d(Radians.of(Math.PI)))), // also rough estimate
                    this.robot.getIntakeRollers().intake())));

    // driver.leftTrigger().onTrue(Commands.print("Super Boost"));

    driver // Auto Drive to Outpost
        .rightBumper()
        .whileTrue(
            this.robot
                .getSwerveDrive()
                .driveTo(
                    new Pose2d(
                        0.6, 0.65, new Rotation2d(Radians.of(Math.PI))))); // also a rough estimate

    // driver.rightTrigger().onTrue(Commands.print("Boost"));

    // Dump fuel
    driver
        .leftStick()
        .whileTrue(
            Commands.parallel(
                this.robot.getIntakeRollers().outtake(), this.robot.getHopper().dump()));

    // Intake and drive to fuel clump
    driver.rightStick().whileTrue(driveToClump.driveToClump());

    // Intake without driving
    driver
        .start()
        .whileTrue(this.robot.getIntakeRollers().intake()); // this might be switched with back

    // Manual climb controls
    operator.a().onTrue(robot.getClimb().descend()); // Lower climb
    operator.b().onTrue(robot.getClimb().pullUp()); // Lift robot
    operator.y().onTrue(robot.getClimb().elevate()); // Raise climb

    // Unjam hopper
    operator.leftBumper().whileTrue(robot.getHopper().unjam());

    // Disable shooting
    operator.leftTrigger().whileTrue(robot.getShooterRollers().shoot(RotationsPerSecond.of(0)));

    // Toggle fine control mode
    operator
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  fineControl = !fineControl;
                }));

    // Force shooting
    operator.rightTrigger().whileTrue(Commands.print("Force Shoot"));

    // Pass fuel to alliance zone
    operator.back().whileTrue(Commands.print("Pass Left")); // this might be switched with start
    operator.start().whileTrue(Commands.print("Pass Right")); // this might be switched with back

    // Fine control
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
