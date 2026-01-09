package com.team6962.lib.swerve.motion;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.team6962.lib.math.SwerveKinematicsUtil;
import com.team6962.lib.math.WheelMath;
import com.team6962.lib.phoenix.control.PositionControlRequest;
import com.team6962.lib.swerve.MotionSwerveDrive;
import com.team6962.lib.swerve.config.DriveMotorConstants;
import com.team6962.lib.swerve.config.SteerMotorConstants;
import com.team6962.lib.swerve.module.SwerveModule;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * A swerve motion that locks the wheels in an X pattern to resist being pushed.
 * 
 * <p>LockMotion orients the swerve modules into an X configuration where each wheel
 * points toward or away from the center of the robot. This configuration makes it
 * very difficult to push the robot in any direction, which is when the robot
 * is being defended to prevent opponents from pushing the robot.
 * 
 * <p>The lock pattern uses position control on both drive and steer motors to
 * actively hold the wheels at their target positions. The target positions are
 * captured when the motion first starts, with steer angles set to form the X pattern.
 */
public class LockMotion implements SwerveMotion {
    /** The swerve drive this motion controls. */
    private final MotionSwerveDrive swerveDrive;
    
    /** The target positions for each module in the X pattern. */
    private SwerveModulePosition[] targetPositions;

    /**
     * Creates a new LockMotion for the specified swerve drive.
     * 
     * @param swerveDrive The swerve drive to lock
     */
    public LockMotion(MotionSwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    /**
     * Updates the swerve modules to hold the X pattern lock position.
     * 
     * <p>On the first update, this method captures the current drive positions
     * and sets the steer angles to form an X pattern (-45°, 45°, 45°, -45° for
     * front-left, front-right, back-left, back-right respectively).
     * 
     * <p>Subsequent updates apply position control to hold all modules at their
     * target positions.
     * 
     * @param deltaTimeSeconds The time since the last update (unused)
     */
    @Override
    public void update(double deltaTimeSeconds) {
        if (targetPositions == null) {
            targetPositions = swerveDrive.getModulePositions();

            for (int i = 0; i < targetPositions.length; i++) {
                targetPositions[i].angle = Rotation2d.fromDegrees(-45 + i * 90);
                targetPositions[i] = SwerveKinematicsUtil.optimizeRelativePosition(targetPositions[i], targetPositions[i].angle.getMeasure());
            }
        }

        double updateFrequencyHz = swerveDrive.getConstants().Timing.ControlLoopFrequency.in(Hertz);
        boolean useTimesync = swerveDrive.getConstants().Timing.TimesyncControlRequests;

        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            SwerveModule module = swerveDrive.getModules()[i];

            Distance drivePosition = Meters.of(targetPositions[i].distanceMeters);
            Angle steerAngle = targetPositions[i].angle.getMeasure();

            DriveMotorConstants driveConstants = swerveDrive.getConstants().DriveMotor;
            SteerMotorConstants steerConstants = swerveDrive.getConstants().SteerMotor;

            module.setControl(
                new PositionControlRequest(WheelMath.toAngular(drivePosition, swerveDrive.getConstants().getWheelRadius(i)).in(Rotations))
                    .withMotionProfileType(driveConstants.PositionControlMotionProfile)
                    .withOutputType(driveConstants.OutputType)
                    .withSlot(driveConstants.PositionSlot)
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
     * Logs telemetry data for this lock motion.
     * 
     * @param basePath The base path for telemetry logging
     */
    @Override
    public void logTelemetry(String basePath) {
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }
        
        DogLog.log(basePath + "Type", "LockMotion");
    }
}
