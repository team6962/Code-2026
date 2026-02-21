package frc.robot.auto;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.ShooterHoodConstants;
import frc.robot.subsystems.visualizer.RobotVisualizationConstants;

public class AutoPassing {
  private RobotContainer robot;

  // Coordinates are not real, they are just for testing in simulation right now
  private final Translation2d LEFT_PASS_TARGET = new Translation2d(1.3, 6.866);
  private final Translation2d RIGHT_PASS_TARGET = new Translation2d(0, 0);

  public AutoPassing(RobotContainer robot) {
    this.robot = robot;
  }

  public Command passLeft() {
    return passTo(LEFT_PASS_TARGET);
  }

  public Command passRight() {
    return passTo(RIGHT_PASS_TARGET);
  }

  private Command passTo(Translation2d target) {
    return Commands.sequence(
        // Move turret first
        robot
            .getTurret()
            .moveTo(
                () -> {
                  // Get the Translation2d from shooter to the left side of the field
                  return target
                      .minus( // Calculate the translational difference between the target and
                          // shooter
                          robot
                              .getSwerveDrive()
                              .getPosition2d() // Get robot position
                              .plus( // Add the small offset between the center of the robot and the
                                  // shooter location
                                  new Transform2d(
                                      RobotVisualizationConstants.shooterTranslation
                                          .toTranslation2d(),
                                      Rotation2d.kZero))
                              .getTranslation())
                      .getAngle()
                      .getMeasure() // Convert the Translation2d into an Angle
                      .minus(
                          robot
                              .getSwerveDrive()
                              .getHeading()); // Subtract the robot's current heading to get the
                  // angle that the turret should rotate to
                }),

        // Retract shooter hood
        robot.getShooterHood().moveTo(ShooterHoodConstants.MAX_ANGLE),

        // Activate the shooter rollers
        robot
            .getShooterRollers()
            .shoot(RotationsPerSecond.of(1)) // arbitrary shooting speed for now
        );
  }
}
