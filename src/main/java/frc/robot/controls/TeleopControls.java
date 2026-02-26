package frc.robot.controls;

import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.auto.DriveFixedShooter;
import frc.robot.subsystems.intakeextension.IntakeExtensionConstants;
import frc.robot.subsystems.shooterrollers.ShooterRollersConstants;

public class TeleopControls {
  private RobotContainer robot;
  private DriveFixedShooter driveFixedShooter;
  //   private AutoClimb autoClimb;
  //   private DriveToClump driveToClump;
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  private boolean fineControl = false;

  public TeleopControls(RobotContainer robot) {
    this.robot = robot;
    this.driveFixedShooter = new DriveFixedShooter(robot);
    // this.autoClimb = new AutoClimb(robot);
    // this.driveToClump = new DriveToClump(robot);

    DogLog.log("TeleopControls/IntakeFineControl", fineControl); // Initial log so that the folder shows up
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
            robot.getSwerveDrive(), robot.getConstants().getTeleopSwerveConstants());

    teleopEnabledTrigger.whileTrue(teleopSwerveCommand);

    // Configure operator controls and automated driver controls

    driver
        .a()
        .whileTrue(driveFixedShooter.driveToClosestShotPosition(DriveFixedShooter.HUB_RADIUS));
    // Driver Y resets heading (configured by XBoxTeleopSwerveCommand)
    // Driver right trigger is boost (configured by XBoxTeleopSwerveCommand)
    // Driver left trigger is super boost (configured by XBoxTeleopSwerveCommand)

    // // Auto Climb and Unclimb
    // driver.b().onTrue(autoClimb.climb());
    // driver.x().onTrue(autoClimb.unclimb());

    // // Auto Depot
    // driver
    //     .leftBumper()
    //     .whileTrue(
    //         Commands.sequence(
    //             this.robot
    //                 .getSwerveDrive()
    //                 .driveTo(
    //                     new Pose2d(
    //                         1.518,
    //                         5.947,
    //                         new Rotation2d(
    //                             Radians.of(
    //                                 Math.PI)))), // rough position estimate based on simulation,
    // not
    //             // exact
    //             this.robot.getIntakeExtension().extend(),
    //             Commands.parallel(
    //                 this.robot
    //                     .getSwerveDrive()
    //                     .driveTo(
    //                         new Pose2d(
    //                             0.546,
    //                             5.947,
    //                             new Rotation2d(Radians.of(Math.PI)))), // also rough estimate
    //                 this.robot.getIntakeRollers().intake())));

    // driver // Auto Drive to Outpost
    //     .rightBumper()
    //     .whileTrue(
    //         this.robot
    //             .getSwerveDrive()
    //             .driveTo(
    //                 new Pose2d(
    //                     0.6, 0.65, new Rotation2d(Radians.of(Math.PI))))); // also a rough
    // estimate

    // Dump fuel
    driver
        .leftStick()
        .whileTrue(
            Commands.parallel(
                this.robot.getIntakeRollers().outtake(), this.robot.getHopper().dump()));

    // Intake and drive to fuel clump
    // driver.rightStick().whileTrue(driveToClump.driveToClump());

    // Intake without driving
    driver
        .start()
        .whileTrue(this.robot.getIntakeRollers().intake()); // this might be switched with back

    // // Manual climb controls
    // operator.a().onTrue(robot.getClimb().descend()); // Lower climb
    // operator.b().onTrue(robot.getClimb().pullUp()); // Lift robot
    // operator.y().onTrue(robot.getClimb().elevate()); // Raise climb

    // Unjam hopper
    operator.leftBumper().whileTrue(robot.getHopper().unjam());

    // Disable shooting
    // operator.leftTrigger().whileTrue(robot.getShooterRollers().shoot(RotationsPerSecond.of(0)));

    // Toggle fine control mode
    operator
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  fineControl = !fineControl;
                  DogLog.log("TeleopControls/IntakeFineControl", fineControl);
                }));

    // Shoot
    operator
        .rightTrigger()
        .whileTrue(
            robot.getShooterRollers().shoot(() -> ShooterRollersConstants.FIXED_FLYWHEEL_VELOCITY));

    // // Pass fuel to alliance zone
    // operator.back().whileTrue(Commands.print("Pass Left")); // this might be switched with start
    // operator.start().whileTrue(Commands.print("Pass Right")); // this might be switched with back

    // // Fine control
    // operator
    //     .povUp()
    //     .and(() -> fineControl)
    //     .whileTrue(
    //
    // this.robot.getShooterHood().moveAtVoltage(ShooterHoodConstants.FINE_CONTROL_VOLTAGE));

    // operator
    //     .povDown()
    //     .and(() -> fineControl)
    //     .whileTrue(
    //         this.robot
    //             .getShooterHood()
    //             .moveAtVoltage(ShooterHoodConstants.FINE_CONTROL_VOLTAGE.unaryMinus()));

    // operator
    //     .povLeft()
    //     .and(() -> fineControl)
    //     .whileTrue(this.robot.getTurret().moveAtVoltage(TurretConstants.FINE_CONTROL_VOLTAGE));

    // operator
    //     .povLeft()
    //     .and(() -> fineControl)
    //     .whileTrue(
    //         this.robot
    //             .getTurret()
    //             .moveAtVoltage(TurretConstants.FINE_CONTROL_VOLTAGE.unaryMinus()));

    operator
        .axisGreaterThan(Axis.kLeftX.value, 0.5)
        .and(() -> fineControl)
        .whileTrue(
            this.robot
                .getIntakeExtension()
                .moveAtVoltage(IntakeExtensionConstants.FINE_CONTROL_VOLTAGE));

    operator
        .axisLessThan(Axis.kLeftX.value, -0.5)
        .and(() -> fineControl)
        .whileTrue(
            this.robot
                .getIntakeExtension()
                .moveAtVoltage(IntakeExtensionConstants.FINE_CONTROL_VOLTAGE.unaryMinus()));

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

    // // Intake extension and retraction
    Trigger intakeRetract = operator.rightStick().or(driver.back());
    Trigger intakeExtend =
        intakeRetract.negate().and(RobotState::isTeleop).and(RobotState::isEnabled);

    intakeRetract.whileTrue(robot.getIntakeExtension().retract());
    intakeExtend.onTrue(
        robot.getIntakeExtension().extend().alongWith(robot.getIntakeRollers().intake()));

    Trigger load =
        new Trigger(() -> RobotState.isTeleop() && RobotState.isEnabled())
            .and(() -> !fineControl)
            .and(driver.leftStick().negate())
            .and(operator.leftBumper().negate());

    load.whileTrue(robot.getHopper().load());

    // Climb retraction
    // Command autodescend = robot.getClimb().descend();
    // Trigger climbRetract =
    //     new Trigger(() -> TeleopSwerveCommand.isClearToOverride(robot.getClimb(), autodescend))
    //         .and(RobotState::isTeleop)
    //         .and(RobotState::isEnabled);

    // climbRetract.onTrue(robot.getClimb().descend());
  }
}
