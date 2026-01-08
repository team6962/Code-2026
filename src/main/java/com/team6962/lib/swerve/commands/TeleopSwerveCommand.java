package com.team6962.lib.swerve.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.CommandSwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Abstract base class for commands that use input from controllers to control
 * a swerve drive in teleoperated mode. This base class handles the execution of
 * the desired velocity, only requiring subclasses to implement the logic for
 * computing that desired velocity.
 * <p>
 * The command automatically schedules separate translation and rotation
 * commands that read from {@link #getDrivenVelocity()}, allowing automation
 * commands to override translation and/or rotation while this command continues
 * to control axes that are not overridden.
 */
public abstract class TeleopSwerveCommand extends Command {
    private CommandSwerveDrive swerveDrive;

    /**
     * Constructs a TeleopSwerveCommand that controls the given swerve drive.
     * This constructor sets up triggers that automatically schedule translation
     * and rotation commands when this command is scheduled and no
     * non-default command is using those subsystems.
     *
     * @param swerveDrive The swerve drive subsystem to control
     */
    public TeleopSwerveCommand(CommandSwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        Command rotationCommand = swerveDrive.driveRotation(() -> RadiansPerSecond.of(getDrivenVelocity().omegaRadiansPerSecond)).repeatedly();
        
        Command translationCommand = swerveDrive.driveTranslation(() -> new TranslationalVelocity(getDrivenVelocity())).repeatedly();

        Trigger rotationTrigger = new Trigger(() -> isScheduled() && isClearToOverride(swerveDrive.useRotation(), rotationCommand));
        Trigger translationTrigger = new Trigger(() -> isScheduled() && isClearToOverride(swerveDrive.useTranslation(), translationCommand));

        rotationTrigger.whileTrue(rotationCommand);
        translationTrigger.whileTrue(translationCommand);
    }

    /**
     * Gets the swerve drive subsystem being controlled by this command.
     *
     * @return The swerve drive subsystem
     */
    protected CommandSwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Computes the desired field-relative velocity for the swerve drive based
     * on the current driver inputs. Subclasses must implement this method to
     * provide the velocity computation logic.
     *
     * @return The desired field-relative chassis speeds
     */
    protected abstract ChassisSpeeds getDrivenVelocity();

    /**
     * Checks if it is okay to replace the currently active command on a
     * subsystem, and have the given command take control instead. A subsystem
     * is clear to override if it has no current command, has only its default
     * command running, or is already running the specified command.
     *
     * @param subsystem The subsystem to check
     * @param command   The command that wants to take control
     * @return True if the subsystem's active command can be safely overridden
     */
    public static boolean isClearToOverride(Subsystem subsystem, Command command) {
        return subsystem.getCurrentCommand() == null || subsystem.getCurrentCommand() == subsystem.getDefaultCommand()
            || subsystem.getCurrentCommand() == command;
    }
}
