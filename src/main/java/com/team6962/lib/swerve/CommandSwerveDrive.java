package com.team6962.lib.swerve;

import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.commands.DriveToStateCommand;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.motion.SwerveMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandSwerveDrive extends MotionSwerveDrive {
    private Subsystem translation = new SubsystemBase() {};
    private Subsystem rotation = new SubsystemBase() {};

    public CommandSwerveDrive(DrivetrainConstants constants) {
        super(constants);
    }

    public Subsystem useTranslation() {
        return translation;
    }

    public Subsystem useRotation() {
        return rotation;
    }

    public Subsystem[] useMotion() {
        return new Subsystem[] {translation, rotation};
    }

    public Set<Subsystem> useMotionSet() {
        return Set.of(translation, rotation);
    }

    public boolean hasTranslationCommand() {
        return translation.getCurrentCommand() == null || translation.getCurrentCommand() == translation.getDefaultCommand();
    }

    public boolean hasRotationCommand() {
        return rotation.getCurrentCommand() == null || rotation.getCurrentCommand() == rotation.getDefaultCommand();
    }

    public Command runTranslation(Runnable toRun) {
        return Commands.run(toRun, translation);
    }

    public Command runRotation(Runnable toRun) {
        return Commands.run(toRun, rotation);
    }

    public Command runMotion(Runnable toRun) {
        return Commands.run(toRun, translation, rotation);
    }

    public Command driveVelocity(Supplier<ChassisSpeeds> velocity) {
        return runMotion(() -> applyVelocityMotion(velocity.get()));
    }

    public Command driveTranslation(Supplier<TranslationalVelocity> velocity) {
        return runTranslation(() -> applyVelocityMotion(velocity.get()));
    }

    public Command driveRotation(Supplier<AngularVelocity> angularVelocity) {
        return runRotation(() -> applyVelocityMotion(angularVelocity.get()));
    }

    public Command driveMotion(SwerveMotion motion, Function<SwerveMotion, SwerveMotion> update) {
        return runMotion(() -> applyMotion(update.apply(motion)));
    }

    public Command driveMotion(SwerveMotion motion) {
        return runMotion(() -> applyMotion(motion));
    }

    public Command neutral() {
        return runMotion(() -> applyNeutralMotion(null));
    }

    public Command brake() {
        return runMotion(() -> applyNeutralMotion(NeutralModeValue.Brake));
    }

    public Command coast() {
        return runMotion(() -> applyNeutralMotion(NeutralModeValue.Coast));
    }

    public Command lock() {
        return runMotion(() -> applyLockMotion());
    }

    public DriveToStateCommand driveTo(DriveToStateCommand.State target) {
        return new DriveToStateCommand(this, target);
    }

    public DriveToStateCommand driveTo(Pose2d pose, ChassisSpeeds velocity) {
        return driveTo(new DriveToStateCommand.State(pose, velocity));
    }

    public DriveToStateCommand driveTo(Pose2d pose) {
        return driveTo(new DriveToStateCommand.State(pose));
    }

    public DriveToStateCommand driveTo(Translation2d translation, TranslationalVelocity translationalVelocity) {
        return driveTo(new DriveToStateCommand.State(translation, translationalVelocity));
    }

    public DriveToStateCommand driveTo(Translation2d translation) {
        return driveTo(new DriveToStateCommand.State(translation));
    }

    public DriveToStateCommand driveTo(Angle angle, AngularVelocity angularVelocity) {
        return driveTo(new DriveToStateCommand.State(angle, angularVelocity));
    }

    public DriveToStateCommand driveTo(Angle angle) {
        return driveTo(new DriveToStateCommand.State(angle));
    }

    public void latePeriodic() {
        clearMotion();
    }
}
