package com.team6962.lib.swerve.simulation;

import java.util.Arrays;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Gyroscope;
import com.team6962.lib.swerve.module.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveDriveSim {
    private SwerveModuleSim[] moduleSims;
    private OdometrySim odometrySim;
    private GyroscopeSim gyroscopeSim;

    public SwerveDriveSim(
        DrivetrainConstants constants,
        SimulationConfig simulationConfig,
        SwerveDriveKinematics kinematics,
        SwerveModule[] modules,
        Gyroscope gyroscope
    ) {
        this.moduleSims = Arrays.stream(modules).map(module -> new SwerveModuleSim(simulationConfig, module)).toArray(SwerveModuleSim[]::new);
        this.odometrySim = new OdometrySim(kinematics, moduleSims);
        this.gyroscopeSim = new GyroscopeSim(constants, gyroscope);
    }

    public void update(double deltaTimeSeconds) {
        // Update each module simulation
        for (SwerveModuleSim moduleSim : moduleSims) {
            moduleSim.update(deltaTimeSeconds);
        }

        // Update odometry simulation
        odometrySim.update();

        // Update gyroscope simulation based on odometry change
        gyroscopeSim.addYaw(odometrySim.getDeltaYaw());
    }
}
