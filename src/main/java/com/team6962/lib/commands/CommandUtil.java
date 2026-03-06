package com.team6962.lib.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Utility class for command-related helper functions. */
public class CommandUtil {
  /**
   * Checks if it is okay to replace the currently active command on a subsystem, and have the given
   * command take control instead. A subsystem is clear to override if it has no current command,
   * has only its default command running, or is already running the specified command.
   *
   * @param subsystem The subsystem to check
   * @param command The command that wants to take control
   * @return True if the subsystem's active command can be safely overridden
   */
  public static boolean isClearToOverride(Subsystem subsystem, Command command) {
    return subsystem.getCurrentCommand() == null
        || subsystem.getCurrentCommand() == subsystem.getDefaultCommand()
        || subsystem.getCurrentCommand() == command;
  }
}
