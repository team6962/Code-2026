package frc.robot.auto;

import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import java.util.Set;

public class IntakeAssist {
  private RobotContainer robot;
  private XBoxTeleopSwerveCommand xBoxTeleopSwerveCommand;

  public IntakeAssist(RobotContainer robot, XBoxTeleopSwerveCommand xBoxTeleopSwerveCommand) {
    this.robot = robot;
    this.xBoxTeleopSwerveCommand = xBoxTeleopSwerveCommand;
  }

  /** Adjusts the robot's velocity to assist with intake alignment */
  public Command adjustVelocity() {
    return Commands.defer(
        () -> {
          return Commands.run(
              () -> {
                robot
                    .getSwerveDrive()
                    .driveVelocity(
                        () -> {
                          Translation2d fuelPosition =
                              robot.getFuelLocalization().getClumpPosition();

                          Translation2d error =
                              fuelPosition.minus(
                                  robot.getSwerveDrive().getPosition2d().getTranslation());

                          double errorX = error.getX();
                          double errorY = error.getY();
                          double errorNorm = error.getNorm();

                          ChassisSpeeds robotVelocity = xBoxTeleopSwerveCommand.getDrivenVelocity();

                          double velocityX = robotVelocity.vxMetersPerSecond;
                          double velocityY = robotVelocity.vyMetersPerSecond;
                          double robotSpeed = Math.hypot(velocityX, velocityY);

                          double kP = 0.5; // Not tuned

                          double maxAssist = 1.0; // Not tuned

                          if (errorNorm < 1e-3) return robotVelocity;
                          if (robotSpeed < 1e-3) return robotVelocity;
                          if (!robot.getIntakeExtension().isExtended()) return robotVelocity;

                          double perpendicularDistanceToFuel =
                              (errorX * velocityY - errorY * velocityX) / robotSpeed;

                          double assistScale =
                              Math.max(0, errorX * velocityX + errorY * velocityY)
                                  / (robotSpeed * errorNorm);

                          double speedScale =
                              Math.min(
                                  1.0,
                                  robotSpeed
                                      / 2.0); // 2.0 m/s should be tuned to the speed at where the
                          // assist
                          // should be at full strength

                          double assist =
                              Math.max(
                                  -maxAssist,
                                  Math.min(
                                      maxAssist,
                                      kP * perpendicularDistanceToFuel * assistScale * speedScale));

                          double correctedVelocityX = (-velocityY / robotSpeed) * assist;
                          double correctedVelocityY = (velocityX / robotSpeed) * assist;

                          return new ChassisSpeeds(
                              velocityX + correctedVelocityX,
                              velocityY + correctedVelocityY,
                              robotVelocity.omegaRadiansPerSecond);
                        });
              },
              robot.getSwerveDrive().useTranslation(),
              robot.getSwerveDrive().useRotation());
        },
        Set.of(
            robot.getSwerveDrive().useTranslation(),
            robot.getSwerveDrive().useRotation(),
            robot.getFuelLocalization()));
  }
}
