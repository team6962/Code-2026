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

public class LockMotion implements SwerveMotion {
    private final MotionSwerveDrive swerveDrive;
    private SwerveModulePosition[] targetPositions;

    public LockMotion(MotionSwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

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

    @Override
    public void logTelemetry(String basePath) {
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }
        
        DogLog.log(basePath + "Type", "LockMotion");
    }
}
