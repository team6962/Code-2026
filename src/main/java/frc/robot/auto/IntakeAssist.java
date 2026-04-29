package frc.robot.auto;

import com.team6962.lib.swerve.commands.XBoxTeleopSwerveCommand;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class IntakeAssist {
  private RobotContainer robot;
  private XBoxTeleopSwerveCommand xBoxTeleopSwerveCommand;

  public IntakeAssist(RobotContainer robot) {
    this.robot = robot;
    this.xBoxTeleopSwerveCommand = new XBoxTeleopSwerveCommand(robot.getSwerveDrive(), robot.getConstants().getTeleopSwerveConstants());
  }

  /** Adjusts the robot's velocity to assist with intake alignment
   * It calculates an assist velocity perpendicular to the robot's current velocity,
   * based on the distance to the fuel clump and how much the robot is moving towards it, 
   * and adds that to the robot's velocity output.
   */
  public Command adjustVelocity() {
    return Commands.run(
        () -> {
          robot
              .getSwerveDrive()
              .driveVelocity(
                  () -> {
                    Translation2d fuelPosition = robot.getFuelLocalization().getClumpPosition();

                    Translation2d error =
                        fuelPosition.minus(robot.getSwerveDrive().getPosition2d().getTranslation());

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

                    double distanceToFuelPerpendicularToVelocity =
                        (errorX * velocityY - errorY * velocityX) / robotSpeed;

                    double assistScale =
                        Math.max(0, errorX * velocityX + errorY * velocityY)
                            / (robotSpeed
                                * errorNorm); // Scale assist based on how much the robot is moving
                    // towards the fuel, so that it doesn't assist when
                    // the robot is moving away from the fuel

                    double speedScale =
                        Math.min(
                            1.0,
                            robotSpeed
                                / 2.0); // 2.0 m/s should be tuned to the speed at where the assist
                    // should be at full strength

                    double assist =
                        Math.max(
                            -maxAssist,
                            Math.min(
                                maxAssist,
                                kP
                                    * distanceToFuelPerpendicularToVelocity
                                    * assistScale
                                    * speedScale)); // Assist is proportional to the distance to the
                    // fuel perpendicular to the velocity, scaled by
                    // how much the robot is moving towards the fuel
                    // and the overall speed of the robot, and
                    // capped at maxAssist

                    double correctedVelocityX = (-velocityY / robotSpeed) * assist;
                    double correctedVelocityY = (velocityX / robotSpeed) * assist;
                    // The assist is applied perpendicular to the velocity vector, in the direction
                    // that reduces the error
                    DogLog.log("IntakeAssist/VelocityAdjustment", new ChassisSpeeds(
                        velocityX + correctedVelocityX,
                        velocityY + correctedVelocityY,
                        robotVelocity.omegaRadiansPerSecond));
                    return new ChassisSpeeds(
                        velocityX + correctedVelocityX,
                        velocityY + correctedVelocityY,
                        robotVelocity.omegaRadiansPerSecond);

                  });
        },
        robot.getSwerveDrive().useTranslation(),
        robot.getSwerveDrive().useRotation());
  }
}
