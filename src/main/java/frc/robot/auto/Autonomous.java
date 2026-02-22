package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hopper.Hopper;

public class Autonomous {
  private RobotContainer robot;
  private DriveToClump driveToClump;
  private Hopper hopper;
  private AutoClimb autoClimb;

  public Autonomous(RobotContainer robot) {
    this.robot = robot;
    this.driveToClump = new DriveToClump(robot);
    this.hopper = robot.getHopper();
    this.autoClimb = new AutoClimb(robot);
  }

  public Command rightPreloadClimbAutoWithVision() {
    return Commands.sequence(
        this.robot
            .getSwerveDrive()
            .driveTo(
                new Pose2d(
                    8.253,
                    1.008,
                    new Rotation2d(
                        Radians.of(
                            Math.PI
                                / 2))), // rough position estimate based on simulation, not exact
                new ChassisSpeeds(
                    0, 0.5, 0)), // not exact chassis speeds), // rough position estimate based on
        // simulation, not exact
        this.driveToClump.driveToClump().until(() -> hopper.isFull()),
        this.robot
            .getSwerveDrive()
            .driveTo(
                new Pose2d(
                    Inches.of(181.56).in(Meters),
                    Inches.of(25.62).in(Meters),
                    new Rotation2d(
                        Radians.of(
                            -Math.PI
                                / 2))), // rough position estimate based on simulation, not exact
                new ChassisSpeeds(0, 0.5, 0)), // not exact chassis speeds
        this.autoClimb.climb());
  }
}
