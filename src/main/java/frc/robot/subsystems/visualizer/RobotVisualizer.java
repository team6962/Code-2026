package frc.robot.subsystems.visualizer;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.ShooterHoodConstants;

/** Displays the articulated components on the robot in AdvantageScope. */
public class RobotVisualizer extends SubsystemBase {
  private RobotContainer robot;

  /**
   * Creates a new RobotVisualizer.
   *
   * @param robot The robot container, used to get the positions of the various subsystems to
   *     visualize
   */
  public RobotVisualizer(RobotContainer robot) {
    this.robot = robot;
  }

  @Override
  public void periodic() {
    // Get robot state
    Angle turretAngle = robot.getTurret().getPosition();
    Angle hoodAngle = robot.getShooterHood().getPosition();
    Distance climbPosition = robot.getClimb().getPosition();
    Distance intakePosition = robot.getIntakeExtension().getPosition();
    SwerveModulePosition[] modulePositions = robot.getSwerveDrive().getModulePositions();

    Pose3d[] poses = new Pose3d[9];

    // Swerve module poses
    for (int i = 0; i < modulePositions.length; i++) {
      Pose2d pose2d =
          new Pose2d(
              robot.getSwerveDrive().getConstants().Structure.WheelBase.div(2).in(Meters)
                  * (i < 2 ? 1 : -1),
              robot.getSwerveDrive().getConstants().Structure.TrackWidth.div(2).in(Meters)
                  * (i % 2 == 0 ? 1 : -1),
              modulePositions[i].angle);

      poses[i] = new Pose3d(pose2d);
    }

    // Shooter and hood pose
    Pose3d shooterPose =
        new Pose3d(
            RobotVisualizationConstants.shooterTranslation,
            new Rotation3d(0, 0, turretAngle.in(Radians)));
    Pose3d shooterRelativeHoodPose =
        new Pose3d(
            RobotVisualizationConstants.hoodTranslation,
            new Rotation3d(0, hoodAngle.minus(ShooterHoodConstants.MIN_ANGLE).in(Radians), 0));
    Pose3d hoodPose = shooterPose.plus(shooterRelativeHoodPose.minus(new Pose3d()));

    poses[4] = shooterPose;
    poses[8] = hoodPose;

    // Intake and extending hopper wall poses
    double intakeHorizontalDistance =
        intakePosition.in(Meters) * Math.cos(RobotVisualizationConstants.intakeAngle.in(Radians));
    double intakeVerticalDistance =
        intakePosition.in(Meters) * Math.sin(RobotVisualizationConstants.intakeAngle.in(Radians));

    poses[5] = new Pose3d(new Translation3d(intakeHorizontalDistance, 0, 0), new Rotation3d());
    poses[6] =
        new Pose3d(
            new Translation3d(intakeHorizontalDistance, 0, intakeVerticalDistance),
            new Rotation3d());

    // Climb pose
    poses[7] = new Pose3d(new Translation3d(0, 0, climbPosition.in(Meters)), new Rotation3d());

    // Log the poses to NetworkTables for visualization in AdvantageScope
    DogLog.log("RobotVisualizer/ArticulatedComponents", poses);
  }
}
