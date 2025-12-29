package com.team6962.lib.swerve.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Arrays;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Gyroscope;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroscopeSim {
    private DrivetrainConstants constants;
    private Pigeon2SimState gyroSim;
    private SwerveModuleSim[] moduleSims;
    private SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    public GyroscopeSim(DrivetrainConstants constants, Gyroscope gyro, SwerveModuleSim[] modulesSims) {
        this.constants = constants;
        this.gyroSim = gyro.getPigeon().getSimState();
        this.kinematics = constants.Structure.getKinematics();
        this.moduleSims = modulesSims;

        refreshModulePositions();
    }

    private void refreshModulePositions() {
        for (int i = 0; i < moduleSims.length; i++) {
            modulePositions[i] = moduleSims[i].getPosition();
        }
    }

    public void update(double deltaTimeSeconds) {
        SwerveModulePosition[] previousPositions = Arrays.copyOf(modulePositions, modulePositions.length);

        refreshModulePositions();

        Twist2d twist = kinematics.toTwist2d(previousPositions, modulePositions);

        gyroSim.addYaw(Radians.of(twist.dtheta));
        gyroSim.setAngularVelocityZ(Radians.of(twist.dtheta).div(Seconds.of(deltaTimeSeconds)));
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
