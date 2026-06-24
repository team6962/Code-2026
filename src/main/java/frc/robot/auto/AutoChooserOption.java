package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;

public class AutoChooserOption {
  public final Command command;
  public final boolean recommended;

  public AutoChooserOption(Command command, boolean recommended) {
    this.command = command;
    this.recommended = recommended;
  }
}
