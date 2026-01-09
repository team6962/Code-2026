package com.team6962.lib.swerve.motion;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.NeutralOut;
import com.team6962.lib.math.SwerveKinematicsUtil;
import com.team6962.lib.math.WheelMath;
import com.team6962.lib.phoenix.control.PositionControlRequest;
import com.team6962.lib.phoenix.control.VelocityControlRequest;
import com.team6962.lib.swerve.MotionSwerveDrive;
import com.team6962.lib.swerve.config.DriveMotorConstants;
import com.team6962.lib.swerve.config.SteerMotorConstants;
import com.team6962.lib.swerve.module.SwerveModule;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * A swerve motion that drives the robot at a specified field-relative velocity.
 * 
 * <p>VelocityMotion converts field-relative chassis speeds into individual swerve module
 * states and applies velocity control to the drive motors and position control to the
 * steer motors. Key features include:
 * <ul>
 *   <li>Field-relative velocity control (velocities are relative to the field, not the robot)</li>
 *   <li>Automatic wheel speed desaturation to respect motor limits</li>
 *   <li>Steer angle optimization to minimize wheel rotation</li>
 *   <li>Velocity reduction when wheels are misaligned</li>
 *   <li>Automatic braking when commanded velocity is near zero</li>
 * </ul>
 * 
 * <p>VelocityMotion supports fusion with other VelocityMotions that have non-overlapping
 * components. For example, a translation-only motion can be fused with a rotation-only
 * motion to create a combined motion with both components.
 */
public class VelocityMotion implements SwerveMotion {
    /** The target field-relative chassis speeds. */
    private final ChassisSpeeds velocity;
    
    /** The swerve drive this motion controls. */
    private final MotionSwerveDrive swerveDrive;
    
    /** Whether this motion has a non-zero translation component. */
    private final boolean hasTranslation;
    
    /** Whether this motion has a non-zero rotation component. */
    private final boolean hasRotation;

    /**
     * Creates a new VelocityMotion with the specified field-relative velocity.
     * 
     * @param velocity The target field-relative chassis speeds
     * @param swerveDrive The swerve drive to control
     */
    public VelocityMotion(ChassisSpeeds velocity, MotionSwerveDrive swerveDrive) {
        this.velocity = velocity;
        this.hasTranslation = Math.abs(velocity.vxMetersPerSecond) > 0.01 || Math.abs(velocity.vyMetersPerSecond) > 0.01;
        this.hasRotation = Math.abs(velocity.omegaRadiansPerSecond) > 0.01;
        this.swerveDrive = swerveDrive;
    }

    /**
     * Fuses this velocity motion with another velocity motion.
     * 
     * <p>Fusion is only allowed when the motions have non-overlapping components
     * (e.g., one has translation only and the other has rotation only).
     * 
     * @param other The motion to fuse with
     * @return A new VelocityMotion with combined velocities
     * @throws IllegalArgumentException If both motions have overlapping translation or rotation
     */
    @Override
    public SwerveMotion fuseWith(SwerveMotion other) {
        if (other instanceof VelocityMotion otherVelocityMotion) {
            if (hasTranslation && otherVelocityMotion.hasTranslation || hasRotation && otherVelocityMotion.hasRotation) {
                throw new IllegalArgumentException("Cannot fuse two VelocityMotions with overlapping translation or rotation components.");
            }

            return new VelocityMotion(
                SwerveKinematicsUtil.addChassisSpeeds(
                    velocity,
                    otherVelocityMotion.velocity
                ),
                swerveDrive
            );
        }

        throw new IllegalArgumentException("Cannot fuse a " + getClass().getSimpleName() + " with a " + other.getClass().getSimpleName());
    }

    /**
     * Updates the swerve modules to drive at the target velocity.
     * 
     * @param deltaTimeSeconds The time since the last update
     */
    @Override
    public void update(double deltaTimeSeconds) {
        if (Math.abs(velocity.omegaRadiansPerSecond) < 0.01 && Math.abs(velocity.vxMetersPerSecond) < 0.01 && Math.abs(velocity.vyMetersPerSecond) < 0.01) {
            brake();
            return;
        }

        ChassisSpeeds robotRelativeVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(velocity, new Rotation2d(swerveDrive.getHeading()));

        SwerveModuleState[] states = swerveDrive.getKinematics().toSwerveModuleStates(robotRelativeVelocity);

        if (!swerveDrive.getConstants().Driving.MaxLinearVelocity.isEquivalent(MetersPerSecond.of(0)) &&
            !swerveDrive.getConstants().Driving.MaxAngularVelocity.isEquivalent(RadiansPerSecond.of(0))) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                states,
                robotRelativeVelocity,
                swerveDrive.getConstants().DriveMotor.MaxVelocity,
                swerveDrive.getConstants().Driving.MaxLinearVelocity,
                swerveDrive.getConstants().Driving.MaxAngularVelocity
            );
        } else {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                states,
                swerveDrive.getConstants().DriveMotor.MaxVelocity
            );
        }

        double updateFrequencyHz = swerveDrive.getConstants().Timing.ControlLoopFrequency.in(Hertz);
        boolean useTimesync = swerveDrive.getConstants().Timing.TimesyncControlRequests;
        DriveMotorConstants driveConstants = swerveDrive.getConstants().DriveMotor;
        SteerMotorConstants steerConstants = swerveDrive.getConstants().SteerMotor;

        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            SwerveModule module = swerveDrive.getModules()[i];
            SwerveModuleState state = states[i];
            Angle currentAngle = module.getSteerMechanism().getPosition();

            state = SwerveKinematicsUtil.optimize(state, currentAngle);
            state = SwerveKinematicsUtil.decreaseVelocityIfMisaligned(state, currentAngle);

            LinearVelocity driveVelocity = MetersPerSecond.of(state.speedMetersPerSecond);
            Angle steerAngle = state.angle.getMeasure();

            module.setControl(
                new VelocityControlRequest(
                        WheelMath.toAngular(driveVelocity, swerveDrive.getConstants().getWheelRadius(i)).in(RotationsPerSecond))
                    .withMotionProfileType(driveConstants.VelocityControlMotionProfile)
                    .withOutputType(driveConstants.OutputType)
                    .withSlot(driveConstants.VelocitySlot)
                    .withUpdateFreqHz(updateFrequencyHz)
                    .withUseTimesync(useTimesync)
                    .toControlRequest(),
                new PositionControlRequest(steerAngle.in(Rotations))
                    .withMotionProfileType(steerConstants.PositionControlMotionProfile)
                    .withOutputType(steerConstants.OutputType)
                    .withSlot(steerConstants.PositionSlot)
                    .withUpdateFreqHz(updateFrequencyHz)
                    .withUseTimesync(useTimesync)
                    .toControlRequest()
            );
        }
    }

    /**
     * Logs telemetry data for this velocity motion.
     * 
     * @param basePath The base path for telemetry logging
     */
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

    /**
     * Sets all motors to neutral/brake mode
     */
    private void brake() {
        NeutralOut neutralOut = new NeutralOut()
            .withUpdateFreqHz(swerveDrive.getConstants().Timing.ControlLoopFrequency)
            .withUseTimesync(swerveDrive.getConstants().Timing.TimesyncControlRequests);

        for (SwerveModule module : swerveDrive.getModules()) {
            module.setControl(neutralOut, neutralOut);
        }
    }
}
