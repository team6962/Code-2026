package com.team6962.lib.swerve.commands;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.team6962.lib.control.TranslationController;
import com.team6962.lib.control.TrapezoidalController;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.CommandSwerveDrive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToStateCommand extends Command {
    private CommandSwerveDrive swerveDrive;
    private State target;
    private TranslationController translationController;
    private TrapezoidalController headingController;

    public static class State {
        public Translation2d translation;
        public TranslationalVelocity translationalVelocity;
        public Angle angle;
        public AngularVelocity angularVelocity;

        public State(Translation2d translation, TranslationalVelocity translationalVelocity, Angle angle, AngularVelocity angularVelocity) {
            this.translation = translation;
            this.translationalVelocity = translationalVelocity;
            this.angle = angle;
            this.angularVelocity = angularVelocity;
        }
    }

    public DriveToStateCommand(CommandSwerveDrive swerveDrive, State target) {
        this.swerveDrive = swerveDrive;
        this.target = target;

        if (target.translation != null) {
            translationController = new TranslationController(
                swerveDrive.getConstants().Driving.TranslationFeedbackKP,
                swerveDrive.getConstants().Driving.TranslationFeedbackKI,
                swerveDrive.getConstants().Driving.TranslationFeedbackKD,
                swerveDrive.getConstants().Driving.getTranslationConstraints(),
                Hertz.of(50)
            );
        }

        if (target.angle != null) {
            headingController = new TrapezoidalController(
                swerveDrive.getConstants().Driving.AngleFeedbackKP,
                swerveDrive.getConstants().Driving.AngleFeedbackKI,
                swerveDrive.getConstants().Driving.AngleFeedbackKD,
                swerveDrive.getConstants().Driving.getRotationConstraints(),
                Hertz.of(50)
            );
        }
    }

    @Override
    public void initialize() {
        if (translationController != null) {
            translationController.setProfile(
                swerveDrive.getPosition().getTranslation(),
                swerveDrive.getTranslationalVelocity(),
                target.translation,
                target.translationalVelocity
            );
        }

        if (headingController != null) {
            headingController.setProfile(
                new TrapezoidProfile.State(
                    swerveDrive.getYaw().in(Radians),
                    swerveDrive.getYawVelocity().in(RadiansPerSecond)
                ),
                new TrapezoidProfile.State(
                    target.angle.in(Radians),
                    target.angularVelocity.in(RadiansPerSecond)
                )
            );
        }
    }

    @Override
    public void execute() {
        if (translationController != null) {
            TranslationalVelocity outputTranslationalVelocity = translationController.calculate(
                swerveDrive.getPosition().getTranslation(),
                swerveDrive.getTranslationalVelocity()
            );

            swerveDrive.applyVelocityMotion(outputTranslationalVelocity);
        }

        if (headingController != null) {
            AngularVelocity outputAngularVelocity = RadiansPerSecond.of(headingController.calculate(new TrapezoidProfile.State(
                swerveDrive.getYaw().in(Radians),
                swerveDrive.getYawVelocity().in(RadiansPerSecond)
            )));

            swerveDrive.applyVelocityMotion(outputAngularVelocity);
        }
    }

    @Override
    public boolean isFinished() {
        return (translationController == null || translationController.isFinished()) &&
               (headingController == null || headingController.isFinished());
    }
}
