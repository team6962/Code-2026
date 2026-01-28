package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveStraightAuto {
  private RobotContainer robot;

  public DriveStraightAuto(RobotContainer robot) {
    this.robot = robot;
  }

  public Command getCommand() {
    return robot
        .getSwerveDrive()
        .driveVelocity(
            () ->
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    new ChassisSpeeds(1, 0, 0), new Rotation2d(robot.getSwerveDrive().getYaw())));
  }
}
