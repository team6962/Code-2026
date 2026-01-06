package com.team6962.lib.swerve.commands;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.team6962.lib.control.TranslationController;
import com.team6962.lib.control.TrapezoidalController;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.CommandSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to drive a swerve drive to a specified state using trapezoidal motion
 * profiling and PID for translation and rotation.
 */
public class DriveToStateCommand extends Command {
    /**
     * The swerve drive subsystem to control.
     */
    private CommandSwerveDrive swerveDrive;

    /**
     * The target state to drive to.
     */
    private State target;

    /**
     * Controller for translation motion.
     */
    private TranslationController translationController;

    /**
     * Controller for rotational motion.
     */
    private TrapezoidalController headingController;

    /**
     * Whether to finish automatically when the target is reached.
     */
    private boolean finishWhenReached = true;

    /**
     * Class representing the target state for the drive command.
     */
    public static class State {
        /**
         * The target translation position. If null, the robot translation
         * will not be controlled.
         */
        public Translation2d translation;

        /**
         * The target translational velocity.
         */
        public TranslationalVelocity translationalVelocity;

        /**
         * The target heading angle. If null, the robot rotation will not be
         * controlled.
         */
        public Angle angle;

        /**
         * The target angular velocity.
         */
        public AngularVelocity angularVelocity;

        /**
         * Constructs a State object with the specified parameters.
         * @param translation           The target translation position
         * @param translationalVelocity The target translational velocity
         * @param angle                 The target heading angle
         * @param angularVelocity       The target angular velocity
         */
        public State(Translation2d translation, TranslationalVelocity translationalVelocity, Angle angle, AngularVelocity angularVelocity) {
            this.translation = translation;
            this.translationalVelocity = translationalVelocity;
            this.angle = angle;
            this.angularVelocity = angularVelocity;
        }

        /**
         * Constructs a State object with the specified parameters.
         * 
         * @param pose The target pose containing translation and angle
         * @param velocity The target chassis speeds containing translational
         *                 and angular velocity
         */
        public State(Pose2d pose, ChassisSpeeds velocity) {
            this(
                pose.getTranslation(),
                new TranslationalVelocity(velocity),
                pose.getRotation().getMeasure(),
                RadiansPerSecond.of(velocity.omegaRadiansPerSecond)
            );
        }

        /**
         * Constructs a State object that stops the robot at the specified
         * pose.
         * 
         * @param pose The target pose containing translation and angle
         */
        public State(Pose2d pose) {
            this(pose, new ChassisSpeeds(0, 0, 0));
        }

        /**
         * Constructs a State object with no rotation control.
         * @param translation           The target translation position
         * @param translationalVelocity The target translational velocity
         */
        public State(Translation2d translation, TranslationalVelocity translationalVelocity) {
            this(translation, translationalVelocity, null, null);
        }

        /**
         * Constructs a State object that stops the robot at the specified
         * translation.
         * @param translation The target translation position
         */
        public State(Translation2d translation) {
            this(translation, new TranslationalVelocity(0, 0));
        }

        /**
         * Constructs a State object with no translation control.
         * @param angle           The target heading angle
         * @param angularVelocity The target angular velocity
         */
        public State(Angle angle, AngularVelocity angularVelocity) {
            this(null, null, angle, angularVelocity);
        }

        /**
         * Constructs a State object that stops the robot at the specified
         * heading.
         * @param angle The target heading angle
         */
        public State(Angle angle) {
            this(angle, RadiansPerSecond.of(0));
        }
    }

    /**
     * Constructs a DriveToStateCommand.
     * @param swerveDrive The swerve drive subsystem to control
     * @param target      The target state to drive to
     */
    public DriveToStateCommand(CommandSwerveDrive swerveDrive, State target) {
        this.swerveDrive = swerveDrive;
        this.target = target;

        // Initialize translation and rotation controllers with PID constants
        // and motion profile constraints
        if (target.translation != null) {
            translationController = new TranslationController(
                swerveDrive.getConstants().Driving.TranslationFeedbackKP,
                swerveDrive.getConstants().Driving.TranslationFeedbackKI,
                swerveDrive.getConstants().Driving.TranslationFeedbackKD,
                swerveDrive.getConstants().Driving.getTranslationConstraints(),
                Hertz.of(50)
            );

            addRequirements(swerveDrive.useTranslation());
        }

        if (target.angle != null) {
            headingController = new TrapezoidalController(
                swerveDrive.getConstants().Driving.AngleFeedbackKP,
                swerveDrive.getConstants().Driving.AngleFeedbackKI,
                swerveDrive.getConstants().Driving.AngleFeedbackKD,
                swerveDrive.getConstants().Driving.getRotationConstraints(),
                Hertz.of(50)
            );

            addRequirements(swerveDrive.useRotation());
        }
    }

    @Override
    public void initialize() {
        // Create motion profiles for translation and rotation controllers
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
        // Calculate and apply velocity commands for translation and rotation
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
        return finishWhenReached &&
               (translationController == null || translationController.isFinished()) &&
               (headingController == null || headingController.isFinished());
    }

    /**
     * Sets whether to finish automatically when the motion profiles finish
     * 
     * @param finishWhenReached True to finish when reached, false to continue
     *                          running the command
     */
    public void setFinishWhenReached(boolean finishWhenReached) {
        this.finishWhenReached = finishWhenReached;
    }

    /**
     * Sets whether to finish automatically when the motion profiles finish
     * 
     * @param finishWhenReached True to finish when reached, false to continue
     *                          running the command
     * @return This DriveToStateCommand for chaining
     */
    public DriveToStateCommand withFinishWhenReached(boolean finishWhenReached) {
        setFinishWhenReached(finishWhenReached);
        return this;
    }
}
