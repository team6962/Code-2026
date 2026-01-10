package com.team6962.lib.swerve.util;

import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Localization;
import com.team6962.lib.swerve.localization.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldLogger implements SwerveComponent {
    private DrivetrainConstants constants;

    private Field2d field = new Field2d();
    private boolean isInitialized = false;

    private Localization localization;
    private Odometry odometry;

    private Pose2d previousRobotPose;
    
    public FieldLogger(DrivetrainConstants constants, Localization localization, Odometry odometry) {
        this.constants = constants;
        this.localization = localization;
        this.odometry = odometry;
    }

    @Override
    public void logTelemetry(String basePath) {
        if (!isInitialized) {
            SmartDashboard.putData(field);
            isInitialized = true;
        }

        if (previousRobotPose != null) {
            Pose2d updatedRobotPose = field.getRobotPose();

            if (!previousRobotPose.equals(updatedRobotPose)) {
                localization.resetPosition(updatedRobotPose);
            }
        }

        Pose2d robotPose = localization.getPosition();

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
