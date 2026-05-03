package frc.robot.auto;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;

public class IntakeAssist {
  private RobotContainer robot;
  private double kP = 0.5; // Not tuned
  private double maxAssist = 1.0; // Not tuned
  private double maxAssistSpeed =
      2.0; // Not tuned, the speed at which the assist should be at full strength

  public IntakeAssist(RobotContainer robot) {
    this.robot = robot;
    DogLog.tunable("IntakeAssist/kP", kP, value -> value = kP);
    DogLog.tunable("IntakeAssist/maxAssist", maxAssist, value -> value = maxAssist);
    DogLog.tunable("IntakeAssist/maxAssistSpeed", maxAssistSpeed, value -> value = maxAssistSpeed);
  }

  /**
   * Adjusts the robot's velocity to assist with intake alignment It calculates an assist velocity
   * perpendicular to the robot's current velocity, based on the distance to the fuel clump and how
   * much the robot is moving towards it, and adds that to the robot's velocity output.
   */
  public ChassisSpeeds calculateAdjustedVelocity(ChassisSpeeds currentVelocity) {

    Translation2d fuelPosition = robot.getFuelLocalization().getClumpPosition();
    if (fuelPosition == null || !robot.getIntakeExtension().isExtended()) return currentVelocity;

    Translation2d error =
        fuelPosition.minus(robot.getSwerveDrive().getPosition2d().getTranslation());

    double errorX = error.getX();
    double errorY = error.getY();
    double errorNorm = error.getNorm();

    double velocityX = currentVelocity.vxMetersPerSecond;
    double velocityY = currentVelocity.vyMetersPerSecond;
    double robotSpeed = Math.hypot(velocityX, velocityY);

    if (errorNorm < 1e-3 || robotSpeed < 1e-3) return currentVelocity;

    double distanceToFuelPerpendicularToVelocity =
        (errorX * velocityY - errorY * velocityX) / robotSpeed;

    double assistScale =
        Math.max(0, errorX * velocityX + errorY * velocityY)
            / (robotSpeed * errorNorm); // Scale assist based on how much the robot is moving
    // towards the fuel, so that it doesn't assist when
    // the robot is moving away from the fuel

    double speedScale = Math.min(1.0, robotSpeed / maxAssistSpeed);

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

    double correctionVelocityX = (-velocityY / robotSpeed) * assist;
    double correctionVelocityY = (velocityX / robotSpeed) * assist;
    // The assist is applied perpendicular to the velocity vector, in the direction
    // that reduces the error
    return new ChassisSpeeds(
        velocityX + correctionVelocityX,
        velocityY + correctionVelocityY,
        currentVelocity.omegaRadiansPerSecond); // Return the original velocity plus the assist
  }
}
