package com.team6962.lib.swerve.motion;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.team6962.lib.math.SwerveKinematicsUtil;
import com.team6962.lib.math.WheelMath;
import com.team6962.lib.swerve.MotionSwerveDrive;
import com.team6962.lib.swerve.module.SwerveModule;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;

public class TwistToTargetMotion implements SwerveMotion {
    private final Pose2d target;
    private final MotionSwerveDrive swerveDrive;
    private Twist2d twist;

    public TwistToTargetMotion(Pose2d target, MotionSwerveDrive swerveDrive) {
        this.target = target;
        this.swerveDrive = swerveDrive;
    }

    @Override
    public void logTelemetry(String basePath) {
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }

        DogLog.log(basePath + "Type", "TwistToTargetMotion");
        DogLog.log(basePath + "TargetPose", target);
        DogLog.log(basePath + "Twist", twist);
    }

    @Override
    public void update(double deltaTimeSeconds) {
        int moduleCount = swerveDrive.getModules().length;

        SwerveModulePosition[] currentPositions = swerveDrive.getModulePositions();

        Pose2d currentPose = swerveDrive.getPosition();

        twist = currentPose.log(target);

        SwerveModuleState[] positionDeltasAsStates = swerveDrive.getKinematics().toSwerveModuleStates(new ChassisSpeeds(
            twist.dx,
            twist.dy,
            twist.dtheta
        ));

        SwerveModulePosition[] positionDeltas = new SwerveModulePosition[moduleCount];
        double[] wheelSpeedRatios = new double[moduleCount];

        for (int i = 0; i < moduleCount; i++) {
            positionDeltas[i] = new SwerveModulePosition(
                positionDeltasAsStates[i].speedMetersPerSecond,
                positionDeltasAsStates[i].angle
            );

            wheelSpeedRatios[i] = Math.abs(positionDeltasAsStates[i].speedMetersPerSecond);
        }

        double greatestWheelSpeed = 0.0;

        for (double wheelSpeedRatio : wheelSpeedRatios) {
            if (wheelSpeedRatio > greatestWheelSpeed) {
                greatestWheelSpeed = wheelSpeedRatio;
            }
        }

        double[] velocityConstraints = new double[moduleCount];
        double[] accelerationConstraints = new double[moduleCount];
        SwerveModulePosition[] targetPositions = new SwerveModulePosition[moduleCount];

        for (int i = 0; i < moduleCount; i++) {
            velocityConstraints[i] = wheelSpeedRatios[i] * WheelMath.toAngular(
                swerveDrive.getConstants().Driving.PreciseDriveVelocity,
                swerveDrive.getConstants().getWheelRadius(i)
            ).in(RotationsPerSecond);

            accelerationConstraints[i] = wheelSpeedRatios[i] * WheelMath.toAngular(
                swerveDrive.getConstants().Driving.PreciseDriveAcceleration,
                swerveDrive.getConstants().getWheelRadius(i)
            ).in(RotationsPerSecondPerSecond);

            // Optimize the swerve module position delta to minimize steering
            // rotation
            SwerveModule module = swerveDrive.getModules()[i];

            Angle steerAngle = module.getSteerMechanism().getPosition();

            positionDeltas[i] = SwerveKinematicsUtil.optimizeRelativePosition(positionDeltas[i], steerAngle);

            targetPositions[i] = new SwerveModulePosition(
                currentPositions[i].distanceMeters + positionDeltas[i].distanceMeters,
                positionDeltas[i].angle
            );

            // Reduce maximum velocities of wheels that are oriented badly
            double cosineError = SwerveKinematicsUtil.getCosineOfSteerError(positionDeltas[i], steerAngle);

            velocityConstraints[i] *= cosineError;
        }

        double updateFrequencyHz = swerveDrive.getConstants().Timing.ControlLoopFrequency.in(Hertz);
        boolean useTimesync = swerveDrive.getConstants().Timing.TimesyncControlRequests;

        for (int i = 0; i < moduleCount; i++) {
            SwerveModule module = swerveDrive.getModules()[i];

            module.setControl(
                swerveDrive.getConstants().DriveMotor.DynamicallyConstraintedPositionControl.createRequest(
                    WheelMath.toAngular(
                        Meters.of(targetPositions[i].distanceMeters),
                        swerveDrive.getConstants().getWheelRadius(i)
                    ).in(Rotations),
                    velocityConstraints[i],
                    accelerationConstraints[i],
                    0.0, // Jerk constraint is not supported.
                    swerveDrive.getConstants().DriveMotor.PositionSlot,
                    updateFrequencyHz,
                    useTimesync
                ),
                swerveDrive.getConstants().SteerMotor.PositionControl.createRequest(
                    targetPositions[i].angle.getRotations(),
                    swerveDrive.getConstants().SteerMotor.PositionSlot,
                    updateFrequencyHz,
                    useTimesync
                )
            );
        }
    }
}
