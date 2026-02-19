package frc.robot.controls;

import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

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
            robot.getSwerveDrive(), robot.getConstants().getTeleopSwerveConstants());

    teleopEnabledTrigger.whileTrue(teleopSwerveCommand);

    // Configure operator controls and automated driver controls
    driver.a().onTrue(Commands.print("Ping!"));
    operator.a().onTrue(Commands.print("Pong!"));
  }
}
