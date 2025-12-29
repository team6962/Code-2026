package com.team6962.lib.swerve.util;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.team6962.lib.swerve.config.DrivetrainConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldLogger implements SwerveComponent {
    private DrivetrainConstants constants;
    private Field2d field = new Field2d();
    private boolean isInitialized = false;
    private Supplier<Pose2d> robotPoseSupplier;
    Supplier<SwerveModuleState[]> moduleStateSupplier;

    public FieldLogger(DrivetrainConstants constants, Supplier<Pose2d> robotPoseSupplier, Supplier<SwerveModuleState[]> moduleStateSupplier) {
        this.constants = constants;
        this.robotPoseSupplier = robotPoseSupplier;
        this.moduleStateSupplier = moduleStateSupplier;
    }

    @Override
    public void logTelemetry(String basePath) {
        if (!isInitialized) {
            SmartDashboard.putData(field);
        }

        Pose2d robotPose = robotPoseSupplier.get();

        field.setRobotPose(robotPose);

        Pose2d[] modulePoses = new Pose2d[4];
        SwerveModuleState[] moduleStates = moduleStateSupplier.get();

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
