package com.team6962.lib.swerve.simulation;

import java.util.Arrays;

import com.team6962.lib.swerve.config.DrivetrainConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class OdometrySim {
    private SwerveModuleSim[] modules;
    private SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] modulePositions;
    private Pose2d robotPose = new Pose2d();
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    public OdometrySim(
        DrivetrainConstants constants,
        SwerveModuleSim[] moduleSims
    ) {
        modules = moduleSims;
        kinematics = constants.Structure.getKinematics();

        modulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
    }

    public void update(double deltaTimeSeconds) {
        SwerveModulePosition[] previousPositions = Arrays.copyOf(modulePositions, modulePositions.length);

        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getPosition();
        }

        Twist2d twist = kinematics.toTwist2d(previousPositions, modulePositions);

        robotPose = robotPose.exp(twist);

        ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(
            twist.dx / deltaTimeSeconds,
            twist.dy / deltaTimeSeconds,
            twist.dtheta / deltaTimeSeconds
        );
        
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            robotRelativeSpeeds,
            robotPose.getRotation()
        );
    }

    public Pose2d getPosition() {
        return robotPose;
    }

    public ChassisSpeeds getVelocity() {
        return chassisSpeeds;
    }
}
