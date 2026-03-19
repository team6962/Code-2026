package com.team6962.lib.swerve.commands;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.team6962.lib.control.MotionProfile;
import com.team6962.lib.control.ProfiledController;
import com.team6962.lib.control.TranslationController;
import com.team6962.lib.control.TrapezoidalProfile;
import com.team6962.lib.math.AngleMath;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.CommandSwerveDrive;
import dev.doglog.DogLog;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to drive a swerve drive to a specified state using trapezoidal motion profiling and PID
 * for translation and rotation.
 */
public class DriveToStateCommand extends Command {
  /** The swerve drive subsystem to control. */
  private CommandSwerveDrive swerveDrive;

  /** The target state to drive to. */
  private State target;

  /** Controller for translation motion. */
  private TranslationController translationController;

  /** Controller for rotational motion. */
  private ProfiledController headingController;

  /** Whether to finish automatically when the target is reached. */
  private boolean finishWhenReached = true;

  /** Whether the motion profiles have finished at least once. */
  private boolean motionProfilesFinished = false;

  /** Most recent drivetrain motion profile scale used to create controllers. */
  private double previousMotionConstraintScale = Double.NaN;

  /** Class representing the target state for the drive command. */
  public static class State {
    /** The target translation position. If null, the robot translation will not be controlled. */
    public Translation2d translation;

    /** The target translational velocity. */
    public TranslationalVelocity translationalVelocity;

    /** The target heading angle. If null, the robot rotation will not be controlled. */
    public Angle angle;

    /** The target angular velocity. */
    public AngularVelocity angularVelocity;

    /**
     * Constructs a State object with the specified parameters.
     *
     * @param translation The target translation position
     * @param translationalVelocity The target translational velocity
     * @param angle The target heading angle
     * @param angularVelocity The target angular velocity
     */
    public State(
        Translation2d translation,
        TranslationalVelocity translationalVelocity,
        Angle angle,
        AngularVelocity angularVelocity) {
      this.translation = translation;
      this.translationalVelocity = translationalVelocity;
      this.angle = angle;
      this.angularVelocity = angularVelocity;
    }

    /**
     * Constructs a State object with the specified parameters.
     *
     * @param pose The target pose containing translation and angle
     * @param velocity The target chassis speeds containing translational and angular velocity
     */
    public State(Pose2d pose, ChassisSpeeds velocity) {
      this(
          pose.getTranslation(),
          new TranslationalVelocity(velocity),
          pose.getRotation().getMeasure(),
          RadiansPerSecond.of(velocity.omegaRadiansPerSecond));
    }

    /**
     * Constructs a State object that stops the robot at the specified pose.
     *
     * @param pose The target pose containing translation and angle
     */
    public State(Pose2d pose) {
      this(pose, new ChassisSpeeds(0, 0, 0));
    }

    /**
     * Constructs a State object with no rotation control.
     *
     * @param translation The target translation position
     * @param translationalVelocity The target translational velocity
     */
    public State(Translation2d translation, TranslationalVelocity translationalVelocity) {
      this(translation, translationalVelocity, null, null);
    }

    /**
     * Constructs a State object that stops the robot at the specified translation.
     *
     * @param translation The target translation position
     */
    public State(Translation2d translation) {
      this(translation, new TranslationalVelocity(0, 0));
    }

    /**
     * Constructs a State object with no translation control.
     *
     * @param angle The target heading angle
     * @param angularVelocity The target angular velocity
     */
    public State(Angle angle, AngularVelocity angularVelocity) {
      this(null, null, angle, angularVelocity);
    }

    /**
     * Constructs a State object that stops the robot at the specified heading.
     *
     * @param angle The target heading angle
     */
    public State(Angle angle) {
      this(angle, RadiansPerSecond.of(0));
    }
  }

  /**
   * Constructs a DriveToStateCommand.
   *
   * @param swerveDrive The swerve drive subsystem to control
   * @param target The target state to drive to
   */
  public DriveToStateCommand(CommandSwerveDrive swerveDrive, State target) {
    this.swerveDrive = swerveDrive;
    this.target = target;

    if (target.translation != null) {
      addRequirements(swerveDrive.useTranslation());
    }

    if (target.angle != null) {
      addRequirements(swerveDrive.useRotation());
    }
  }

  @Override
  public void initialize() {
    // Reset state
    motionProfilesFinished = false;

    createControllers();

    // Generate initial motion profiles
    createMotionProfiles();

    DogLog.log(
        "Drivetrain/DriveToState/FinalTarget",
        new Pose2d(target.translation, new Rotation2d(target.angle)));
    DogLog.log("Drivetrain/DriveToState/FinalVelocityX", target.translationalVelocity.x);
    DogLog.log("Drivetrain/DriveToState/FinalVelocityY", target.translationalVelocity.y);
    DogLog.log("Drivetrain/DriveToState/FinalAngularVelocity", target.angularVelocity);
  }

  /** Creates controllers for translation and rotation using the current scaled constraints. */
  private void createControllers() {
    previousMotionConstraintScale = swerveDrive.getMotionConstraintScale();

    if (target.translation != null) {
      translationController =
          new TranslationController(
              swerveDrive.getConstants().Driving.TranslationFeedbackKP,
              swerveDrive.getConstants().Driving.TranslationFeedbackKI,
              swerveDrive.getConstants().Driving.TranslationFeedbackKD,
              swerveDrive.getScaledTranslationConstraints(),
              Hertz.of(50));
    }

    if (target.angle != null) {
      headingController =
          new ProfiledController(
              swerveDrive.getConstants().Driving.AngleFeedbackKP,
              swerveDrive.getConstants().Driving.AngleFeedbackKI,
              swerveDrive.getConstants().Driving.AngleFeedbackKD,
              new TrapezoidalProfile(swerveDrive.getScaledRotationConstraints()),
              Hertz.of(50));
    }
  }

  /** Creates motion profiles for translation and rotation controllers. */
  private void createMotionProfiles() {
    if (translationController != null) {
      translationController.setProfile(
          swerveDrive.getPosition2d().getTranslation(),
          swerveDrive.getTranslationalVelocity(),
          target.translation,
          target.translationalVelocity);
    }

    if (headingController != null) {
      // Ensure the robot doesn't try to rotate more than 180 degrees
      Angle targetAngle = target.angle;

      targetAngle = AngleMath.toDiscrete(targetAngle);
      targetAngle = AngleMath.toContinuous(targetAngle, swerveDrive.getYaw());

      headingController.setProfile(
          new MotionProfile.State(
              swerveDrive.getYaw().in(Radians), swerveDrive.getYawVelocity().in(RadiansPerSecond)),
          new MotionProfile.State(
              targetAngle.in(Radians), target.angularVelocity.in(RadiansPerSecond)));
    }

    if (headingController != null && translationController != null) {
      double maxDuration =
          Math.max(headingController.getDuration(), translationController.getDuration());

      headingController.setDuration(maxDuration);
      translationController.setDuration(maxDuration);
    }
  }

  @Override
  public void execute() {
    if (Math.abs(swerveDrive.getMotionConstraintScale() - previousMotionConstraintScale) > 0.05) {
      createControllers();
      createMotionProfiles();
    }

    // Check if motion profiles have finished
    if ((translationController == null || translationController.isFinished())
        && (headingController == null || headingController.isFinished())) {
      motionProfilesFinished = true;

      // Generate new motion profiles to move closer to the target
      createMotionProfiles();
    }

    // Current target pose and speeds for logging
    Pose2d currentTarget = new Pose2d();
    ChassisSpeeds currentSpeeds = new ChassisSpeeds();

    // Calculate and apply velocity commands for translation and rotation
    if (translationController != null) {
      TranslationalVelocity currentTranslationalVelocity =
          translationController.calculate(
              swerveDrive.getPosition2d().getTranslation(),
              swerveDrive.getTranslationalVelocity(),
              0);

      currentSpeeds =
          new ChassisSpeeds(
              currentTranslationalVelocity.x.in(MetersPerSecond),
              currentTranslationalVelocity.y.in(MetersPerSecond),
              currentSpeeds.omegaRadiansPerSecond);

      TranslationalVelocity nextTranslationalVelocity =
          translationController.calculate(
              swerveDrive.getPosition2d().getTranslation(),
              swerveDrive.getTranslationalVelocity(),
              0.02);

      Vector<N2> acceleration =
          nextTranslationalVelocity
              .toVector()
              .minus(currentTranslationalVelocity.toVector())
              .div(0.02);

      currentTranslationalVelocity =
          currentTranslationalVelocity.plus(
              new TranslationalVelocity(acceleration)
                  .times(swerveDrive.getConstants().Driving.AutoLinearAccelerationScalar));

      swerveDrive.applyVelocityMotion(currentTranslationalVelocity);

      // Add translation to current target pose
      currentTarget =
          new Pose2d(
              translationController.getCurrentTarget(), swerveDrive.getPosition2d().getRotation());
    }

    if (headingController != null) {
      MotionProfile.State angularState =
          new MotionProfile.State(
              swerveDrive.getYaw().in(Radians), swerveDrive.getYawVelocity().in(RadiansPerSecond));

      AngularVelocity currentAngularVelocity =
          RadiansPerSecond.of(headingController.calculate(angularState));

      currentSpeeds =
          new ChassisSpeeds(
              currentSpeeds.vxMetersPerSecond,
              currentSpeeds.vyMetersPerSecond,
              currentAngularVelocity.in(RadiansPerSecond));

      AngularVelocity nextAngularVelocity =
          RadiansPerSecond.of(
              headingController.calculate(
                  new MotionProfile.State(
                      swerveDrive.getYaw().in(Radians),
                      swerveDrive.getYawVelocity().in(RadiansPerSecond)),
                  0.02));

      AngularAcceleration angularAcceleration =
          nextAngularVelocity.minus(currentAngularVelocity).div(Seconds.of(0.02));

      currentAngularVelocity =
          currentAngularVelocity.plus(
              angularAcceleration.times(
                  Seconds.of(swerveDrive.getConstants().Driving.AutoAngularAccelerationScalar)));

      swerveDrive.applyVelocityMotion(currentAngularVelocity);

      // Add rotation to current target pose
      currentTarget =
          new Pose2d(
              currentTarget.getTranslation(),
              Rotation2d.fromRadians(headingController.sample().position));
    }

    // Log current target pose
    swerveDrive.getFieldLogger().getField().getObject("Current Target").setPose(currentTarget);

    DogLog.log("Drivetrain/DriveToState/ProfilePose", currentTarget);
    DogLog.log(
        "Drivetrain/DriveToState/ProfileVelocityX",
        currentSpeeds.vxMetersPerSecond,
        MetersPerSecond);
    DogLog.log(
        "Drivetrain/DriveToState/ProfileVelocityY",
        currentSpeeds.vyMetersPerSecond,
        MetersPerSecond);
    DogLog.log(
        "Drivetrain/DriveToState/ProfileAngularVelocity",
        currentSpeeds.omegaRadiansPerSecond,
        RadiansPerSecond);
  }

  @Override
  public boolean isFinished() {
    return finishWhenReached && motionProfilesFinished;
  }

  /**
   * Sets whether to finish automatically when the motion profiles finish
   *
   * @param finishWhenReached True to finish when reached, false to continue running the command
   */
  public void setFinishWhenReached(boolean finishWhenReached) {
    this.finishWhenReached = finishWhenReached;
  }

  /**
   * Sets whether to finish automatically when the motion profiles finish
   *
   * @param finishWhenReached True to finish when reached, false to continue running the command
   * @return This DriveToStateCommand for chaining
   */
  public DriveToStateCommand withFinishWhenReached(boolean finishWhenReached) {
    setFinishWhenReached(finishWhenReached);
    return this;
  }
}
