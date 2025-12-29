package com.team6962.lib.swerve.motion;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.NeutralOut;
import com.team6962.lib.math.SwerveKinematicsUtil;
import com.team6962.lib.math.WheelMath;
import com.team6962.lib.swerve.MotionSwerveDrive;
import com.team6962.lib.swerve.module.SwerveModule;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;

public class VelocityMotion implements SwerveMotion {
    private final ChassisSpeeds velocity;
    private final MotionSwerveDrive swerveDrive;
    private final boolean hasTranslation;
    private final boolean hasRotation;

    public VelocityMotion(ChassisSpeeds velocity, boolean hasTranslation, boolean hasRotation, MotionSwerveDrive swerveDrive) {
        this.velocity = velocity;
        this.hasTranslation = hasTranslation && (Math.abs(velocity.vxMetersPerSecond) > 0.01 || Math.abs(velocity.vyMetersPerSecond) > 0.01);
        this.hasRotation = hasRotation && Math.abs(velocity.omegaRadiansPerSecond) > 0.01;
        this.swerveDrive = swerveDrive;
    }

    @Override
    public SwerveMotion fuseWith(SwerveMotion other) {
        if (other instanceof VelocityMotion otherVelocityMotion) {
            if (otherVelocityMotion.hasRotation && !hasRotation) {
                return new VelocityMotion(
                    new ChassisSpeeds(
                        velocity.vxMetersPerSecond,
                        velocity.vyMetersPerSecond,
                        otherVelocityMotion.velocity.omegaRadiansPerSecond
                    ),
                    hasTranslation,
                    true,
                    swerveDrive
                );
            } else if (otherVelocityMotion.hasTranslation && !hasTranslation) {
                return new VelocityMotion(
                    new ChassisSpeeds(
                        otherVelocityMotion.velocity.vxMetersPerSecond,
                        otherVelocityMotion.velocity.vyMetersPerSecond,
                        velocity.omegaRadiansPerSecond
                    ),
                    true,
                    hasRotation,
                    swerveDrive
                );
            } else if (!otherVelocityMotion.hasTranslation && !otherVelocityMotion.hasRotation) {
                return this;
            } else if (!hasTranslation && !hasRotation) {
                return otherVelocityMotion;
            }
        }

        return null;
    }

    @Override
    public void update(double deltaTimeSeconds) {
        if (Math.abs(velocity.omegaRadiansPerSecond) < 0.01 && Math.abs(velocity.vxMetersPerSecond) < 0.01 && Math.abs(velocity.vyMetersPerSecond) < 0.01) {
            brake();
            return;
        }

        ChassisSpeeds robotRelativeVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, new Rotation2d(swerveDrive.getHeading()));

        SwerveModuleState[] states = swerveDrive.getKinematics().toSwerveModuleStates(robotRelativeVelocity);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, swerveDrive.getConstants().DriveMotor.MaxVelocity.in(MetersPerSecond));

        double updateFrequencyHz = swerveDrive.getConstants().Timing.ControlLoopFrequency.in(Hertz);
        boolean useTimesync = swerveDrive.getConstants().Timing.TimesyncControlRequests;

        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            SwerveModule module = swerveDrive.getModules()[i];
            SwerveModuleState state = states[i];
            Angle currentAngle = module.getSteerMechanism().getPosition();

            state = SwerveKinematicsUtil.optimize(state, currentAngle);
            state = SwerveKinematicsUtil.cosineCorrect(state, currentAngle);

            LinearVelocity driveVelocity = MetersPerSecond.of(state.speedMetersPerSecond);
            Angle steerAngle = state.angle.getMeasure();

            module.setControl(
                swerveDrive.getConstants().DriveMotor.VelocityControl.createRequest(
                    WheelMath.toAngular(driveVelocity, swerveDrive.getConstants().getWheelRadius(i)).in(RotationsPerSecond),
                    swerveDrive.getConstants().DriveMotor.VelocitySlot,
                    updateFrequencyHz,
                    useTimesync
                ),
                swerveDrive.getConstants().SteerMotor.PositionControl.createRequest(
                    steerAngle.in(Rotations),
                    swerveDrive.getConstants().SteerMotor.PositionSlot,
                    updateFrequencyHz,
                    useTimesync
                )
            );
        }
    }

    @Override
    public void logTelemetry(String basePath) {
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }
            
        DogLog.log(basePath + "Type", "VelocityMotion");
        DogLog.log(basePath + "LinearVelocityX", velocity.vxMetersPerSecond);
        DogLog.log(basePath + "LinearVelocityY", velocity.vyMetersPerSecond);
        DogLog.log(basePath + "AngularVelocity", velocity.omegaRadiansPerSecond);
    }

    private void brake() {
        Frequency updateFrequency = swerveDrive.getConstants().Timing.ControlLoopFrequency;
        boolean timesync = swerveDrive.getConstants().Timing.TimesyncControlRequests;

        for (SwerveModule module : swerveDrive.getModules()) {
            module.setControl(
                new NeutralOut()
                    .withUpdateFreqHz(updateFrequency)
                    .withUseTimesync(timesync),
                new NeutralOut()
                    .withUpdateFreqHz(updateFrequency)
                    .withUseTimesync(timesync)
            );
        }
    }
}
