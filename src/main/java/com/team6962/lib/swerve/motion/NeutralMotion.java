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

/**
 * A swerve motion that sets all motors to a neutral state.
 * 
 * <p>NeutralMotion disables active control of the swerve modules and allows them to
 * either coast freely or actively brake, depending on the configured neutral mode.
 * 
 * <p>The neutral mode can be configured to:
 * <ul>
 *   <li>{@link NeutralModeValue#Coast} - Motors spin freely with no resistance</li>
 *   <li>{@link NeutralModeValue#Brake} - Motors actively resist rotation</li>
 *   <li>{@code null} - Uses the default neutral output behavior specified in
 *                      the motor controller configurations</li>
 * </ul>
 */
public class NeutralMotion implements SwerveMotion {
    /** The swerve drive this motion controls. */
    private final MotionSwerveDrive swerveDrive;
    
    /** The neutral mode to apply (Coast, Brake, or null for default). */
    private final NeutralModeValue neutralMode;
    
    /** The control request to send to all motors. */
    private final ControlRequest request;

    /**
     * Creates a new NeutralMotion with the default neutral mode.
     * 
     * @param swerveDrive The swerve drive to set to neutral
     */
    public NeutralMotion(MotionSwerveDrive swerveDrive) {
        this(swerveDrive, null);
    }

    /**
     * Creates a new NeutralMotion with the specified neutral mode.
     * 
     * @param swerveDrive The swerve drive to set to neutral
     * @param neutralMode The neutral mode (Coast, Brake, or null for default)
     */
    public NeutralMotion(MotionSwerveDrive swerveDrive, NeutralModeValue neutralMode) {
        this.swerveDrive = swerveDrive;
        this.neutralMode = neutralMode;

        double updateFreqHz = swerveDrive.getConstants().Timing.ControlLoopFrequency.in(Hertz);
        boolean useTimesync = swerveDrive.getConstants().Timing.TimesyncControlRequests;

        if (neutralMode == NeutralModeValue.Coast) {
            request = new CoastOut()
                .withUpdateFreqHz(updateFreqHz)
                .withUseTimesync(useTimesync);
        } else if (neutralMode == NeutralModeValue.Brake) {
            request = new StaticBrake()
                .withUpdateFreqHz(updateFreqHz)
                .withUseTimesync(useTimesync);
        } else {
            request = new NeutralOut()
                .withUpdateFreqHz(updateFreqHz)
                .withUseTimesync(useTimesync);
        }
    }

    /**
     * Applies the neutral control request to all swerve modules.
     * 
     * @param deltaTimeSeconds The time since the last update (unused)
     */
    @Override
    public void update(double deltaTimeSeconds) {
        for (SwerveModule module : swerveDrive.getModules()) {
            module.setControl(request, request);
        }
    }

    /**
     * Logs telemetry data for this neutral motion.
     * 
     * @param basePath The base path for telemetry logging
     */
    @Override
    public void logTelemetry(String basePath) {
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }
            
        DogLog.log(basePath + "Type", "NeutralMotion");
        DogLog.log(basePath + "NeutralMode", neutralMode != null ? neutralMode.toString() : "Default");
    }
}
