package frc.robot.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import java.util.Set;

public class DriveToClump {
  private RobotContainer robot;

  public DriveToClump(RobotContainer robot) {
    this.robot = robot;
  }

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
  public Command driveToClump() {
    return Commands.defer(
            () -> {
              Translation2d fuelPosition = robot.getFuelLocalization().getClumpPosition();
              if (fuelPosition == null) {
                return Commands.none();
              }

              Translation2d error =
                  fuelPosition.minus(robot.getSwerveDrive().getPosition2d().getTranslation());

              double maxVelocity =
                  robot
                      .getConstants()
                      .getDrivetrainConstants()
                      .Driving
                      .MaxLinearVelocity
                      .in(MetersPerSecond);
              double maxAcceleration =
                  robot
                      .getConstants()
                      .getDrivetrainConstants()
                      .Driving
                      .MaxLinearAcceleration
                      .in(MetersPerSecondPerSecond);

              double distance = error.getNorm();
              double finalSpeed = Math.min(maxVelocity, Math.sqrt(2 * maxAcceleration * distance));

              ChassisSpeeds finalVelocity =
                  new ChassisSpeeds(
                      error.getX() / distance * finalSpeed,
                      error.getY() / distance * finalSpeed,
                      0);

              return Commands.either(
                  Commands.deadline(
                      robot
                          .getSwerveDrive()
                          .driveTo(new Pose2d(fuelPosition, error.getAngle()), finalVelocity),
                      robot.getIntakeRollers().intake()),
                  Commands.none(),
                  robot.getIntakeExtension()::isExtended);
            },
            Set.of(
                robot.getIntakeExtension(),
                robot.getIntakeRollers(),
                robot.getSwerveDrive().useTranslation(),
                robot.getSwerveDrive().useRotation()))
        .repeatedly();
  }
}
