package frc.robot.auto;

import com.team6962.lib.swerve.CommandSwerveDrive;
import com.team6962.lib.vision.SphereClumpLocalization;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakeextension.IntakeExtension;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import java.util.Set;

public class DriveToClump {
  /**
   * Creates a command that repeatedly attempts to drive the robot to a detected clump of fuel while
   * running the intake rollers when the intake is extended.
   *
   * <p>Behavior:
   *
   * <ul>
   *   <li>On each execution, the command queries the provided {@code fuelClumpLocalization} for the
   *       current clump position.
   *   <li>If no clump position is available (null), the command does nothing for that iteration.
   *   <li>If a clump position is available and the intake extension reports it is extended, the
   *       command runs the intake rollers and commands the swerve drive to drive to the clump
   *       position in parallel.
   *   <li>If a clump position is available but the intake is not extended, the command does nothing
   *       for that iteration.
   *   <li>The composed command is created with {@code Commands.defer(...).repeatedly()}, so the
   *       check-and-act behavior repeats until the returned command is cancelled or interrupted.
   * </ul>
   *
   * @param intakeExtension subsystem that reports whether the intake is extended; used as the
   *     condition to decide whether to intake and drive
   * @param intakeRollers subsystem used to run the intake rollers when approaching a clump
   * @param fuelClumpLocalization provider used to obtain the current Translation2d position of the
   *     detected fuel clump (may return {@code null} if no fuel is visible)
   * @param swerveDrive swerve-drive command provider used to drive to the detected fuel clump
   *     position
   * @return a {@code Command} that repeatedly checks for a clump and, when appropriate, runs the
   *     intake rollers while driving to the fuel clump; returns {@code Commands.none()} for
   *     iterations where no action is taken
   */
  public Command driveToClump(
      IntakeExtension intakeExtension,
      IntakeRollers intakeRollers,
      SphereClumpLocalization fuelClumpLocalization,
      CommandSwerveDrive swerveDrive) {
    return Commands.defer(
            () -> {
              Translation2d fuelPosition = fuelClumpLocalization.getClumpPosition();
              if (fuelPosition == null) {
                return Commands.none();
              }

              return Commands.either(
                  Commands.parallel(
                      intakeRollers.run(intakeRollers::intake), swerveDrive.driveTo(fuelPosition)),
                  Commands.none(),
                  intakeExtension::isExtended);
            },
            Set.of(
                intakeExtension,
                intakeRollers,
                swerveDrive.useTranslation(),
                swerveDrive.useRotation()))
        .repeatedly();
  }
}
