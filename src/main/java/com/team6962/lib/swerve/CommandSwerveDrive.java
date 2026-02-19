package com.team6962.lib.swerve;

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
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * A swerve drive class that provides command-based control of the rotational and translational
 * motion of the drivetrain. Note that this class does not extend {@link SubsystemBase}, so commands
 * do not run on a CommandSwerveDrive directly. Instead, there are two seperate subsystems for
 * controlling translational and rotational motion, accessible via {@link #useTranslation()} and
 * {@link #useRotation()}, respectively.
 */
public class CommandSwerveDrive extends MotionSwerveDrive {
  /** Subsystem for controlling translational motion. */
  private Subsystem translation = new SubsystemBase() {};

  /** Subsystem for controlling rotational motion. */
  private Subsystem rotation = new SubsystemBase() {};

  /**
   * Creates a new CommandSwerveDrive with the given configuration constants.
   *
   * @param constants The drivetrain configuration constants.
   */
  public CommandSwerveDrive(DrivetrainConstants constants) {
    super(constants);
  }

  /**
   * Gets the subsystem used for controlling translational motion. Commands that need to control
   * translation should require this subsystem.
   *
   * @return The translation subsystem
   */
  public Subsystem useTranslation() {
    return translation;
  }

  /**
   * Gets the subsystem used for controlling rotational motion. Commands that need to control
   * rotation should require this subsystem.
   *
   * @return The rotation subsystem
   */
  public Subsystem useRotation() {
    return rotation;
  }

  /**
   * Gets both the translation and rotation subsystems as an array. Commands that need to control
   * both translation and rotation should require both of these subsystems.
   *
   * @return An array containing the translation and rotation subsystems
   */
  public Subsystem[] useMotion() {
    return new Subsystem[] {translation, rotation};
  }

  /**
   * Gets both the translation and rotation subsystems as a set. This is useful for commands that
   * accept a set of subsystem requirements.
   *
   * @return A set containing the translation and rotation subsystems
   */
  public Set<Subsystem> useMotionSet() {
    return Set.of(translation, rotation);
  }

  /**
   * Creates a command that runs the given action repeatedly while requiring only the translation
   * subsystem.
   *
   * @param toRun The action to run each iteration
   * @return A command that runs the action on the translation subsystem
   */
  public Command runTranslation(Runnable toRun) {
    return Commands.run(toRun, translation);
  }

  /**
   * Creates a command that runs the given action repeatedly while requiring only the rotation
   * subsystem.
   *
   * @param toRun The action to run each iteration
   * @return A command that runs the action on the rotation subsystem
   */
  public Command runRotation(Runnable toRun) {
    return Commands.run(toRun, rotation);
  }

  /**
   * Creates a command that runs the given action repeatedly while requiring both the translation
   * and rotation subsystems.
   *
   * @param toRun The action to run each iteration
   * @return A command that runs the action on both subsystems
   */
  public Command runMotion(Runnable toRun) {
    return Commands.run(toRun, translation, rotation);
  }

  /**
   * Creates a command that drives the robot at the velocity provided by the supplier. The command
   * requires both translation and rotation subsystems.
   *
   * @param velocity A supplier that provides the desired chassis speeds
   * @return A command that applies the velocity motion
   */
  public Command driveVelocity(Supplier<ChassisSpeeds> velocity) {
    return runMotion(() -> applyVelocityMotion(velocity.get()));
  }

  /**
   * Creates a command that drives the robot at the specified velocity. The command requires both
   * translation and rotation subsystems.
   *
   * @param velocity The desired chassis speeds
   * @return A command that applies the velocity motion
   */
  public Command drive(ChassisSpeeds velocity) {
    return runMotion(() -> applyVelocityMotion(velocity));
  }

  /**
   * Creates a command that drives the robot's translation at the velocity provided by the supplier.
   * The command only requires the translation subsystem, allowing rotation to be controlled
   * independently.
   *
   * @param velocity A supplier that provides the desired translational velocity
   * @return A command that applies the translational velocity motion
   */
  public Command driveTranslation(Supplier<TranslationalVelocity> velocity) {
    return runTranslation(() -> applyVelocityMotion(velocity.get()));
  }

  /**
   * Creates a command that drives the robot's translation at the specified velocity. The command
   * only requires the translation subsystem, allowing rotation to be controlled independently.
   *
   * @param velocity The desired translational velocity
   * @return A command that applies the translational velocity motion
   */
  public Command drive(TranslationalVelocity velocity) {
    return runTranslation(() -> applyVelocityMotion(velocity));
  }

  /**
   * Creates a command that drives the robot's rotation at the angular velocity provided by the
   * supplier. The command only requires the rotation subsystem, allowing translation to be
   * controlled independently.
   *
   * @param angularVelocity A supplier that provides the desired angular velocity
   * @return A command that applies the rotational velocity motion
   */
  public Command driveRotation(Supplier<AngularVelocity> angularVelocity) {
    return runRotation(() -> applyVelocityMotion(angularVelocity.get()));
  }

  /**
   * Creates a command that drives the robot's rotation at the specified angular velocity. The
   * command only requires the rotation subsystem, allowing translation to be controlled
   * independently.
   *
   * @param angularVelocity The desired angular velocity
   * @return A command that applies the rotational velocity motion
   */
  public Command drive(AngularVelocity angularVelocity) {
    return runRotation(() -> applyVelocityMotion(angularVelocity));
  }

  /**
   * Creates a command that applies a swerve motion, with an update function called each iteration
   * to modify the motion.
   *
   * @param motion The base swerve motion to apply
   * @param update A function that updates the motion each iteration
   * @return A command that applies the updated motions
   */
  public Command driveMotion(SwerveMotion motion, Function<SwerveMotion, SwerveMotion> update) {
    return runMotion(() -> applyMotion(update.apply(motion)));
  }

  /**
   * Creates a command that applies a swerve motion.
   *
   * @param motion The swerve motion to apply
   * @return A command that applies the motion
   */
  public Command driveMotion(SwerveMotion motion) {
    return runMotion(() -> applyMotion(motion));
  }

  /**
   * Creates a command that puts all motors in the configured default neutral mode.
   *
   * @return A command that applies neutral motion
   */
  public Command neutral() {
    return runMotion(() -> applyNeutralMotion(null));
  }

  /**
   * Creates a command that puts all motors in brake mode, actively resisting motion.
   *
   * @return A command that applies brake motion
   */
  public Command brake() {
    return runMotion(() -> applyNeutralMotion(NeutralModeValue.Brake));
  }

  /**
   * Creates a command that puts all motors in coast mode, allowing free spinning.
   *
   * @return A command that applies coast motion
   */
  public Command coast() {
    return runMotion(() -> applyNeutralMotion(NeutralModeValue.Coast));
  }

  /**
   * Creates a command that locks the wheels in an X pattern to prevent the robot from being pushed.
   *
   * @return A command that applies lock motion
   */
  public Command lock() {
    return runMotion(() -> applyLockMotion());
  }

  /**
   * Creates a command that drives the robot to the specified target state using motion profiling
   * and PID.
   *
   * @param target The target state containing position and velocity goals
   * @return A DriveToStateCommand configured with the target
   */
  public DriveToStateCommand driveTo(DriveToStateCommand.State target) {
    return new DriveToStateCommand(this, target);
  }

  /**
   * Creates a command that drives the robot to the specified pose with the specified ending
   * velocity.
   *
   * @param pose The target pose (position and heading)
   * @param velocity The target velocity at the end of the motion
   * @return A DriveToStateCommand configured with the target
   */
  public DriveToStateCommand driveTo(Pose2d pose, ChassisSpeeds velocity) {
    return driveTo(new DriveToStateCommand.State(pose, velocity));
  }

  /**
   * Creates a command that drives the robot to the specified pose, stopping at the target.
   *
   * @param pose The target pose (position and heading)
   * @return A DriveToStateCommand configured with the target
   */
  public DriveToStateCommand driveTo(Pose2d pose) {
    return driveTo(new DriveToStateCommand.State(pose));
  }

  /**
   * Creates a command that drives the robot to the specified translation with the specified ending
   * translational velocity. Rotation is not controlled.
   *
   * @param translation The target translation (x, y position)
   * @param translationalVelocity The target velocity at the end of the motion
   * @return A DriveToStateCommand configured with the target
   */
  public DriveToStateCommand driveTo(
      Translation2d translation, TranslationalVelocity translationalVelocity) {
    return driveTo(new DriveToStateCommand.State(translation, translationalVelocity));
  }

  /**
   * Creates a command that drives the robot to the specified translation, stopping at the target.
   * Rotation is not controlled.
   *
   * @param translation The target translation (x, y position)
   * @return A DriveToStateCommand configured with the target
   */
  public DriveToStateCommand driveTo(Translation2d translation) {
    return driveTo(new DriveToStateCommand.State(translation));
  }

  /**
   * Creates a command that rotates the robot to the specified heading with the specified ending
   * angular velocity. Translation is not controlled.
   *
   * @param angle The target heading angle
   * @param angularVelocity The target angular velocity at the end of th motion
   * @return A DriveToStateCommand configured with the target
   */
  public DriveToStateCommand driveTo(Angle angle, AngularVelocity angularVelocity) {
    return driveTo(new DriveToStateCommand.State(angle, angularVelocity));
  }

  /**
   * Creates a command that rotates the robot to the specified heading, stopping at the target.
   * Translation is not controlled.
   *
   * @param angle The target heading angle
   * @return A DriveToStateCommand configured with the target
   */
  public DriveToStateCommand driveTo(Angle angle) {
    return driveTo(new DriveToStateCommand.State(angle));
  }

  /**
   * Should be called at the end of each periodic cycle to clear the previous motion and begin
   * executing the most recently applied motion.
   */
  public void latePeriodic() {
    updateMotion();
  }
}
