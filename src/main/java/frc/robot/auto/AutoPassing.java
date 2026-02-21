package frc.robot.auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.ShooterHoodConstants;
import frc.robot.subsystems.visualizer.RobotVisualizationConstants;

public class AutoPassing {
  private RobotContainer robot;

  // Coordinates are not real, they are just for testing in simulation right now
  private final Translation2d LEFT_PASS_TARGET = new Translation2d(0, 8);
  private final Translation2d RIGHT_PASS_TARGET = new Translation2d(0, 0);

  public AutoPassing(RobotContainer robot) {
    this.robot = robot;
  }

  /**
   * Calculates the translational difference between the target and shooter
   * @param target The 2D field coordinates of the target
   * @return A Translation2d representing where the target is relative to the shooter
   */
  public Translation2d getTranslationToTarget(Translation2d target) {
    return target.minus(
                        robot
                            .getSwerveDrive()
                            .getPosition2d() // Get robot position
                            .plus( // Add the small offset between the center of the robot and the shooter location
                                new Transform2d(
                                    RobotVisualizationConstants.shooterTranslation
                                        .toTranslation2d(),
                                    Rotation2d.kZero))
                            .getTranslation());
  }

  /**
   * Gets the direct distance from the shooter to a target
   * @param target The 2D field coordinates of the target
   * @return A Distance representing the direct distance from the shooter to the target
   */
  public Distance getDistanceToTarget(Translation2d target) {
    return Meters.of(getTranslationToTarget(target).getNorm());
  }

  public Angle getIdealHoodAngle(Translation2d target) {
    return robot.getShooterFunctions().getHoodAngle(getDistanceToTarget(target));
  }

  public AngularVelocity getIdealFlywheelVelocity(Translation2d target) {
    return robot.getShooterFunctions().getFlywheelVelocity(getDistanceToTarget(target));
  }

  public Command passLeft() {
    return passTo(LEFT_PASS_TARGET);
  }

  public Command passRight() {
    return passTo(RIGHT_PASS_TARGET);
  }

  private Command passTo(Translation2d target) {
    return Commands.parallel(
        // Move turret
        robot
            .getTurret()
            .moveTo(
                () -> {
                  // Get the Translation2d from shooter to the left side of the field
                  return getTranslationToTarget(target)
                        .getAngle()
                        .getMeasure() // Convert the Translation2d into an Angle
                        .minus(
                            robot
                                .getSwerveDrive()
                                .getHeading()); // Subtract the robot's current heading to get the angle that the turret should rotate to
                }),

        // Move shooter hood
        robot.getShooterHood().moveTo(() -> getIdealHoodAngle(target)), // use ShooterFunctions class to calculate correct hood angle

        // Activate the shooter rollers
        robot
            .getShooterRollers()
            .shoot(() -> getIdealFlywheelVelocity(target))
    );
  }
}
