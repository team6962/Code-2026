package com.team6962.lib.swerve.simulation;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.team6962.lib.swerve.localization.Gyroscope;

import edu.wpi.first.units.measure.Angle;

public class GyroscopeSim {
    private Pigeon2SimState gyroSim;

    public GyroscopeSim(Gyroscope gyro) {
        this.gyroSim = gyro.getPigeon().getSimState();
    }

    public void update(Angle deltaYaw) {
        gyroSim.addYaw(deltaYaw);
    }
}
