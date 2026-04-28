package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import java.util.Set;

public class IntakeAssist {
  private RobotContainer robot;

  public IntakeAssist(RobotContainer robot) {
    this.robot = robot;
  }

  public Command adjustVelocity() {
    return Commands.defer(
        () -> {
          Translation2d fuelPosition = robot.getFuelLocalization().getClumpPosition();

          ChassisSpeeds robotVelocity = robot.getSwerveDrive().getVelocity();

          Translation2d error =
              fuelPosition.minus(robot.getSwerveDrive().getPosition2d().getTranslation());

          double errorX = error.getX();
          double errorY = error.getY();

          double velocityX = robotVelocity.vxMetersPerSecond;
          double velocityY = robotVelocity.vyMetersPerSecond;

          double kP = 0.5; // Not tuned

          double maxAssist = 1.0; // Max speed correction in m/s, not tuned

          double robotSpeed = Math.hypot(velocityX, velocityY);

          double perpendicularDistanceToFuel =
              (errorX * velocityY - errorY * velocityX) / robotSpeed;

          if (robotSpeed < 1e-6) return Commands.none();

          if (errorX * velocityX + errorY * velocityY <= 0) return Commands.none();

          if (!robot.getIntakeExtension().isExtended()) return Commands.none();

          double assist = Math.max(-maxAssist, Math.min(maxAssist, kP * perpendicularDistanceToFuel));

          double correctedVelocityX = (-velocityY / robotSpeed) * assist;
          double correctedVelocityY = (velocityX / robotSpeed) * assist;

          ChassisSpeeds correctedVelocity =
              new ChassisSpeeds(correctedVelocityX, correctedVelocityY, 0);

          return Commands.run(() -> robot.getSwerveDrive().driveVelocity(() -> correctedVelocity));
        },
        Set.of(robot.getSwerveDrive().useTranslation(), robot.getSwerveDrive().useRotation()));
  }
}
