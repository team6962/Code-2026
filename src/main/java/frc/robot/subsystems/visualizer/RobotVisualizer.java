package frc.robot.subsystems.visualizer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.team6962.lib.swerve.simulation.MapleSim;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.ShooterHoodConstants;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/** Displays the articulated components on the robot in AdvantageScope. */
public class RobotVisualizer extends SubsystemBase {
  private RobotContainer robot;
  private IntakeSimulation intakeSim;

  /**
   * Creates a new RobotVisualizer.
   *
   * @param robot The robot container, used to get the positions of the various subsystems to
   *     visualize
   */
  public RobotVisualizer(RobotContainer robot) {
    this.robot = robot;

    // Set up intake simulation
    MapleSim mapleSim = robot.getSwerveDrive().getSimulation().getMapleSim();
    intakeSim =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            mapleSim.getSwerveSim(),
            Inches.of(27),
            Inches.of(12),
            IntakeSimulation.IntakeSide.FRONT,
            48);
  }

  private Translation3d getFuelTranslation(int index) {
    double spacing = RobotVisualizationConstants.fuelSpacing.in(Meters);
    Translation3d basePose = new Translation3d(0, -spacing * 1.5, 0.1);

    int length =
        robot.getIntakeExtension().isExtended()
            ? RobotVisualizationConstants.hopperExtendedLength
            : RobotVisualizationConstants.hopperRetractedLength;
    int width = RobotVisualizationConstants.hopperWidth;

    int layer = index / length / width;
    int indexInLayer = index % (length * width);
    int row = indexInLayer / length;
    int column = indexInLayer % length;

    return basePose.plus(new Translation3d(column * spacing, row * spacing, layer * spacing));
  }

  @Override
  public void periodic() {
    // Update intake simulation
    if (robot.getIntakeExtension().isExtended()
        && robot.getIntakeRollers().getAppliedVoltage().in(Volts) > 1.0) {
      intakeSim.startIntake();
    } else {
      intakeSim.stopIntake();
    }

    // Visualize fuel in robot
    int fuelCount = intakeSim.getGamePiecesAmount();

    if (!robot.getIntakeExtension().isExtended()
        && fuelCount > RobotVisualizationConstants.maxRetractedFuel) {
      for (int i = 0; i < fuelCount - RobotVisualizationConstants.maxRetractedFuel; i++) {
        Translation3d fuelTranslation = getFuelTranslation(i);

        robot
            .getSwerveDrive()
            .getSimulation()
            .getMapleSim()
            .getArena()
            .addGamePieceProjectile(
                new RebuiltFuelOnFly(
                    robot.getSwerveDrive().getPosition2d().getTranslation(),
                    fuelTranslation
                        .toTranslation2d()
                        .rotateBy(robot.getSwerveDrive().getPosition2d().getRotation()),
                    robot.getSwerveDrive().getVelocity(),
                    Rotation2d.fromRotations(Math.random()),
                    Meters.of(fuelTranslation.getZ()),
                    MetersPerSecond.of(Math.random() * 2.0 + 2.5),
                    Degrees.of(65.0 + Math.random() * 20.0)));
      }

      intakeSim.setGamePiecesCount(RobotVisualizationConstants.maxRetractedFuel);
      fuelCount = RobotVisualizationConstants.maxRetractedFuel;
    }

    List<Pose3d> additionalFuelPoses = new ArrayList<>();

    for (int i = 0; i < fuelCount; i++) {
      Translation3d fuelTranslation = getFuelTranslation(i);
      additionalFuelPoses.add(new Pose3d(fuelTranslation, new Rotation3d()));
    }

    robot
        .getSwerveDrive()
        .getSimulation()
        .getMapleSim()
        .setHeldFuelPoses(additionalFuelPoses.toArray(Pose3d[]::new));

    DogLog.log("RobotVisualizer/HeldFuelCount", fuelCount);

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
