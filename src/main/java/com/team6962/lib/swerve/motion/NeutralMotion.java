package com.team6962.lib.swerve.motion;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6962.lib.swerve.MotionSwerveDrive;
import com.team6962.lib.swerve.module.SwerveModule;

import dev.doglog.DogLog;

public class NeutralMotion implements SwerveMotion {
    private final MotionSwerveDrive swerveDrive;
    private final NeutralModeValue neutralMode;
    private final ControlRequest request;

    public NeutralMotion(MotionSwerveDrive swerveDrive, NeutralModeValue neutralMode) {
        this.swerveDrive = swerveDrive;
        this.neutralMode = neutralMode;

        double updateFreqHz = swerveDrive.getConstants().Timing.ControlLoopFrequency.in(Hertz);
        boolean useTimesync = swerveDrive.getConstants().Timing.TimesyncControlRequests;

        if (neutralMode == null) {
            request = new NeutralOut()
                .withUpdateFreqHz(updateFreqHz)
                .withUseTimesync(useTimesync);
        } else if (neutralMode == NeutralModeValue.Brake) {
            request = new StaticBrake()
                .withUpdateFreqHz(updateFreqHz)
                .withUseTimesync(useTimesync);
        } else {
            request = new CoastOut()
                .withUpdateFreqHz(updateFreqHz)
                .withUseTimesync(useTimesync);
        }
    }

    @Override
    public void update(double deltaTimeSeconds) {
        for (SwerveModule module : swerveDrive.getModules()) {
            module.setControl(request, request);
        }
    }

    @Override
    public void logTelemetry(String basePath) {
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }
            
        DogLog.log(basePath + "Type", "NeutralMotion");
        DogLog.log(basePath + "NeutralMode", neutralMode != null ? neutralMode.toString() : "Default");
    }
}
