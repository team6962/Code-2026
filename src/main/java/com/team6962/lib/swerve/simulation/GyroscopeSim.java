package com.team6962.lib.swerve.simulation;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Gyroscope;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroscopeSim {
    private DrivetrainConstants constants;
    private Pigeon2SimState gyroSim;

    public GyroscopeSim(DrivetrainConstants constants, Gyroscope gyro) {
        this.constants = constants;
        this.gyroSim = gyro.getPigeon().getSimState();
    }

    public void addYaw(Angle deltaYaw) {
        gyroSim.addYaw(deltaYaw);
    }

    // TODO: Check mount pose adjustment

    public void setYaw(Angle yaw) {
        gyroSim.setRawYaw(yaw.plus(Degrees.of(constants.Gyroscope.DeviceConfiguration.MountPose.MountPoseYaw)));
    }

    public void setPitch(Angle pitch) {
        gyroSim.setPitch(pitch);
    }

    public void setRoll(Angle roll) {
        gyroSim.setRoll(roll);
    }

    public void setYawVelocity(AngularVelocity yawVelocity) {
        gyroSim.setAngularVelocityZ(yawVelocity);
    }

    public void setPitchVelocity(AngularVelocity pitchVelocity) {
        gyroSim.setAngularVelocityY(pitchVelocity);
    }

    public void setRollVelocity(AngularVelocity rollVelocity) {
        gyroSim.setAngularVelocityX(rollVelocity);
    }
}
