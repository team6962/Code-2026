package com.team6962.lib.swerve.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.CommandSwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class TeleopSwerveCommand extends Command {
    private CommandSwerveDrive swerveDrive;

    public TeleopSwerveCommand(CommandSwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        Command rotationCommand = swerveDrive.driveRotation(() -> RadiansPerSecond.of(getDrivenVelocity().omegaRadiansPerSecond)).repeatedly();
        
        Command translationCommand = swerveDrive.driveTranslation(() -> new TranslationalVelocity(getDrivenVelocity())).repeatedly();

        Trigger rotationTrigger = new Trigger(() -> isScheduled() && isClearToOverride(swerveDrive.useRotation(), rotationCommand));
        Trigger translationTrigger = new Trigger(() -> isScheduled() && isClearToOverride(swerveDrive.useTranslation(), translationCommand));

        rotationTrigger.whileTrue(rotationCommand);
        translationTrigger.whileTrue(translationCommand);
    }

    protected CommandSwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    protected abstract ChassisSpeeds getDrivenVelocity();

    public static boolean isClearToOverride(Subsystem subsystem, Command command) {
        return subsystem.getCurrentCommand() == null || subsystem.getCurrentCommand() == subsystem.getDefaultCommand()
            || subsystem.getCurrentCommand() == command;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
