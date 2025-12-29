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

    public VelocityMotion(ChassisSpeeds velocity, MotionSwerveDrive swerveDrive) {
        this.velocity = velocity;
        this.hasTranslation = Math.abs(velocity.vxMetersPerSecond) > 0.01 || Math.abs(velocity.vyMetersPerSecond) > 0.01;
        this.hasRotation = Math.abs(velocity.omegaRadiansPerSecond) > 0.01;
        this.swerveDrive = swerveDrive;
    }

    @Override
    public SwerveMotion fuseWith(SwerveMotion other) {
        if (other instanceof VelocityMotion otherVelocityMotion) {
            if (hasTranslation && otherVelocityMotion.hasTranslation || hasRotation && otherVelocityMotion.hasRotation) {
                throw new IllegalArgumentException("Cannot fuse two VelocityMotions with overlapping translation or rotation components.");
            }

            return new VelocityMotion(
                SwerveKinematicsUtil.sum(
                    velocity,
                    otherVelocityMotion.velocity
                ),
                swerveDrive
            );
        }

        throw new IllegalArgumentException("Cannot fuse a " + getClass().getSimpleName() + " with a " + other.getClass().getSimpleName());
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
