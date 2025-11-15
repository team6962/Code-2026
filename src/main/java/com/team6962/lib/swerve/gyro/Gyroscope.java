package com.team6962.lib.swerve.gyro;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team6962.lib.phoenix.StatusUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Provides an interface to the Pigeon 2.0 gyroscope with methods to get
 * latency-compensated orientations and angular velocities.
 */
public class Gyroscope {
    private GyroscopeConfig config;

    /**
     * The Pigeon 2.0 gyroscope instance. This communicates with the physical
     * gyro hardware over CAN bus.
     */
    private Pigeon2 gyro;

    private StatusSignal<Angle> yawSignal;
    private StatusSignal<AngularVelocity> yawVelocitySignal;

    private StatusSignal<Angle> pitchSignal;
    private StatusSignal<AngularVelocity> pitchVelocitySignal;

    private StatusSignal<Angle> rollSignal;
    private StatusSignal<AngularVelocity> rollVelocitySignal;

    public Gyroscope(GyroscopeConfig config) {
        this.config = config;

        gyro = new Pigeon2(config.canID, config.canBusName);

        StatusUtil.check(gyro.getConfigurator().apply(config.pigeonConfiguration));

        yawSignal = gyro.getYaw();
        yawVelocitySignal = gyro.getAngularVelocityZWorld();

        pitchSignal = gyro.getPitch();
        pitchVelocitySignal = gyro.getAngularVelocityYWorld();

        rollSignal = gyro.getRoll();
        rollVelocitySignal = gyro.getAngularVelocityXWorld();

        BaseStatusSignal.setUpdateFrequencyForAll(config.updateFrequency, getStatusSignals());
    }

    public GyroscopeConfig getConfig() {
        return config;
    }

    public BaseStatusSignal[] getStatusSignals() {
        return new BaseStatusSignal[] {
            yawSignal,
            yawVelocitySignal,
            pitchSignal,
            pitchVelocitySignal,
            rollSignal,
            rollVelocitySignal
        };
    }

    public Pigeon2 getPigeon() {
        return gyro;
    }

    public Angle getYaw() {
        return Rotations.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            yawSignal,
            yawVelocitySignal
        ));
    }

    public Angle getPitch() {
        return Rotations.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            pitchSignal,
            pitchVelocitySignal
        ));
    }

    public Angle getRoll() {
        return Rotations.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            rollSignal,
            rollVelocitySignal
        ));
    }

    public AngularVelocity getYawVelocity() {
        return yawVelocitySignal.getValue();
    }

    public AngularVelocity getPitchVelocity() {
        return pitchVelocitySignal.getValue();
    }

    public AngularVelocity getRollVelocity() {
        return rollVelocitySignal.getValue();
    }

    public void update() {
    }
}
