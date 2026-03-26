package com.team6962.lib.swerve.util;

import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Localization;
import com.team6962.lib.swerve.localization.Odometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Logs the robot's position and swerve module states to a {@link Field2d} widget on Sim
 * GUI/AdvantageScope/Elastic.
 *
 * <p>This component provides two-way interaction with the field visualization: it displays the
 * robot's current pose from localization, and if the user drags the robot in the dashboard, the
 * localization is reset to that position. This enables quick pose correction during testing.
 *
 * <p>The swerve module poses are also displayed as separate objects, showing the wheel positions
 * and orientations relative to the robot chassis.
 */
public class FieldLogger implements SwerveComponent {
  /** Drivetrain configuration. */
  private DrivetrainConstants constants;

  /** The Field2d widget for visualization. */
  private Field2d field = new Field2d();

  /** Whether the field has been published to SmartDashboard. */
  private boolean isPublished = false;

  /** Localization system to read and reset robot pose. */
  private Localization localization;

  /** Odometry system to read module states. */
  private Odometry odometry;

  /** Cached previous pose to detect user-initiated changes. */
  private Pose2d previousRobotPose;

  /** Notifier for periodic field logging updates. */
  private Notifier fieldLoggerNotifier;

  /** Data structure to hold the latest robot and module poses for logging. */
  private LogData logData;

  /**
   * Data structure to hold the robot pose and module poses for logging. This is cloned before being
   * accessed in the logging thread to avoid concurrency issues.
   */
  private static class LogData implements Cloneable {
    /** The robot's current pose on the field. */
    public Pose2d robotPose;

    /** The poses of the swerve modules relative to the robot. */
    public Pose2d[] modulePoses;

    public LogData(Pose2d robotPose, Pose2d[] modulePoses) {
      this.robotPose = robotPose;
      this.modulePoses = modulePoses;
    }

    @Override
    public LogData clone() {
      Pose2d clonedRobotPose = new Pose2d(robotPose.getTranslation(), robotPose.getRotation());
      Pose2d[] clonedModulePoses = null;
      if (modulePoses != null) {
        clonedModulePoses = new Pose2d[modulePoses.length];
        for (int i = 0; i < modulePoses.length; i++) {
          clonedModulePoses[i] =
              new Pose2d(modulePoses[i].getTranslation(), modulePoses[i].getRotation());
        }
      }
      return new LogData(clonedRobotPose, clonedModulePoses);
    }
  }

  /**
   * Creates a new FieldLogger, which logs robot and module poses to NetworkTables.
   *
   * @param constants drivetrain configuration
   * @param localization localization system to read/reset pose
   * @param odometry odometry system to read module states
   */
  public FieldLogger(DrivetrainConstants constants, Localization localization, Odometry odometry) {
    this.constants = constants;
    this.localization = localization;
    this.odometry = odometry;
    fieldLoggerNotifier = new Notifier(this::threadedLog);
    fieldLoggerNotifier.setName("FieldLogger");
    fieldLoggerNotifier.startPeriodic(0.02);
  }

  /**
   * Logs the robot pose and module poses to the Field2d widget. This method is called periodically
   * by the Notifier in a separate thread.
   */
  private void threadedLog() {
    LogData dataCopy;

    synchronized (this) {
      if (logData == null) {
        return;
      }

      dataCopy = logData.clone();
    }

    if (dataCopy.robotPose != null) {
      field.setRobotPose(dataCopy.robotPose);
    }

    if (dataCopy.modulePoses != null) {
      field.getObject("Swerve Modules").setPoses(dataCopy.modulePoses);
    }
  }

  @Override
  public void logTelemetry(String basePath) {
    if (!isPublished) {
      SmartDashboard.putData(field);
      isPublished = true;
    }

    if (RobotBase.isSimulation() && previousRobotPose != null) {
      Pose2d updatedRobotPose = field.getRobotPose();

      if (!previousRobotPose.equals(updatedRobotPose)) {
        localization.resetPosition(updatedRobotPose);
      }
    }

    Pose2d robotPose = localization.getPosition2d();
    Pose2d[] modulePoses = null;

    if (!constants.Timing.MinimizeLogging) {
      modulePoses = new Pose2d[4];
      SwerveModuleState[] moduleStates = odometry.getStates();

      for (int i = 0; i < 4; i++) {
        Pose2d relativePose =
            new Pose2d(
                constants.Structure.WheelBase.div(2).in(Meters) * (i < 2 ? 1 : -1),
                constants.Structure.TrackWidth.div(2).in(Meters) * (i % 2 == 0 ? 1 : -1),
                moduleStates[i].angle);

        modulePoses[i] = robotPose.plus(relativePose.minus(new Pose2d()));
      }
    }

    previousRobotPose = robotPose;

    synchronized (this) {
      logData = new LogData(robotPose, modulePoses);
    }
  }

  /**
   * Gets the Field2d widget being used for visualization. This can be used to customize the field
   * display (e.g. adding trajectories, targets, etc.) in addition to the robot and module poses.
   *
   * @return the Field2d widget for visualization
   */
  public Field2d getField() {
    return field;
  }
}
