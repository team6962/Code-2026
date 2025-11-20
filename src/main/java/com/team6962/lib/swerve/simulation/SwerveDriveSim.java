package com.team6962.lib.swerve.simulation;

import java.util.Arrays;

import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Gyroscope;
import com.team6962.lib.swerve.module.SwerveModule;

public class SwerveDriveSim {
    private SwerveModuleSim[] moduleSims;
    private GyroscopeSim gyroscopeSim;

    public SwerveDriveSim(
        DrivetrainConstants constants,
        SwerveModule[] modules,
        Gyroscope gyroscope
    ) {
        this.moduleSims = Arrays.stream(modules).map(module -> new SwerveModuleSim(module)).toArray(SwerveModuleSim[]::new);
        this.gyroscopeSim = new GyroscopeSim(constants, gyroscope, moduleSims);
    }

    public SwerveModuleSim[] getModules() {
        return moduleSims;
    }

    public GyroscopeSim getGyroscope() {
        return gyroscopeSim;
    }

    public void update(double deltaTimeSeconds) {
        // Update each module simulation
        for (SwerveModuleSim moduleSim : moduleSims) {
            moduleSim.update(deltaTimeSeconds);
        }

        // Update gyroscope simulation
        gyroscopeSim.update();
    }
}
