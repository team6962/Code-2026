package frc.robot.controls;

import static edu.wpi.first.units.Units.Volts;

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
import frc.robot.subsystems.hood.ShooterHoodConstants;

public class TeleopControls {
  private RobotContainer robot;
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  public TeleopControls(RobotContainer robot) {
    this.robot = robot;
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
    driver.b().onTrue(Commands.print("Climb"));
    driver.x().onTrue(Commands.print("Unclimb"));
    // driver.y().onTrue(Commands.print("Reset Heading"));
    driver.leftBumper().whileTrue(Commands.print("Depot"));
    // driver.leftTrigger().onTrue(Commands.print("Super Boost"));
    driver.rightBumper().whileTrue(Commands.print("Outpost"));
    // driver.rightTrigger().onTrue(Commands.print("Boost"));
    driver.leftStick().whileTrue(Commands.print("Dump"));
    driver.back().whileTrue(Commands.print("Retract Intake")); // this might be switched with start
    driver.rightStick().whileTrue(Commands.print("Intake and Drive To Fuel"));
    driver.start().whileTrue(Commands.print("Run Intake!")); // this might be switched with back

    operator.a().onTrue(Commands.print("Lower Climb"));
    operator.b().onTrue(Commands.print("Lift Robot"));
    operator.x().onTrue(Commands.print("Unclimb"));
    operator.y().onTrue(Commands.print("Raise Climb"));
    operator.leftBumper().whileTrue(Commands.print("Unjam"));
    operator.leftTrigger().whileTrue(Commands.print("Disable Shoot"));
    operator.rightBumper().onTrue(Commands.print("Toggle Fine Control"));
    operator.rightTrigger().whileTrue(Commands.print("Force Shoot"));
    operator.leftStick().whileTrue(Commands.print("Dump"));
    operator.back().whileTrue(Commands.print("Pass Left")); // this might be switched with start
    operator.rightStick().whileTrue(Commands.print("Retract Intake"));
    operator.start().whileTrue(Commands.print("Pass Right")); // this might be switched with back
    operator.povUp().whileTrue(this.robot.getShooterHood().moveAtVoltage(ShooterHoodConstants.FINE_CONTROL_VOLTAGE)); //CHECK SIGN
    operator.povDown().whileTrue(this.robot.getShooterHood().moveAtVoltage(ShooterHoodConstants.FINE_CONTROL_VOLTAGE.unaryMinus())); //CHECK SIGN
  }
}
