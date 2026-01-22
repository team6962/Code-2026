package com.team6962.lib.swerve.util;

import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Localization;
import com.team6962.lib.swerve.localization.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Logs the robot's position and swerve module states to a {@link Field2d}
 * widget on Sim GUI/AdvantageScope/Elastic.
 *
 * <p>This component provides two-way interaction with the field visualization:
 * it displays the robot's current pose from localization, and if the user
 * drags the robot in the dashboard, the localization is reset to that
 * position. This enables quick pose correction during testing.
 *
 * <p>The swerve module poses are also displayed as separate objects, showing
 * the wheel positions and orientations relative to the robot chassis.
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

    /**
     * Creates a new FieldLogger, which logs robot and module poses to
     * NetworkTables.
     *
     * @param constants drivetrain configuration
     * @param localization localization system to read/reset pose
     * @param odometry odometry system to read module states
     */
    public FieldLogger(DrivetrainConstants constants, Localization localization, Odometry odometry) {
        this.constants = constants;
        this.localization = localization;
        this.odometry = odometry;
    }

    @Override
    public void logTelemetry(String basePath) {
        if (!isPublished) {
            SmartDashboard.putData(field);
            isPublished = true;
        }

        if (previousRobotPose != null) {
            Pose2d updatedRobotPose = field.getRobotPose();

            if (!previousRobotPose.equals(updatedRobotPose)) {
                localization.resetPosition(updatedRobotPose);
            }
        }

        Pose2d robotPose = localization.getPosition2d();

        previousRobotPose = robotPose;

        field.setRobotPose(robotPose);

        Pose2d[] modulePoses = new Pose2d[4];
        SwerveModuleState[] moduleStates = odometry.getStates();

        for (int i = 0; i < 4; i++) {
            Pose2d relativePose = new Pose2d(
                constants.Structure.WheelBase.div(2).in(Meters) * (i < 2 ? 1 : -1),
                constants.Structure.TrackWidth.div(2).in(Meters) * (i % 2 == 0 ? 1 : -1),
                moduleStates[i].angle
            );

            modulePoses[i] = robotPose.plus(relativePose.minus(new Pose2d()));
        }

        field.getObject("Swerve Modules").setPoses(modulePoses);
    }
}
