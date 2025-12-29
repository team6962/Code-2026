package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveKinematicsUtil {
    public static SwerveModuleState optimize(SwerveModuleState target, Angle steerAngle) {
        LinearVelocity velocity = MetersPerSecond.of(target.speedMetersPerSecond);
        
        Angle targetAngle = target.angle.getMeasure();
        Angle continuousDelta = targetAngle.minus(steerAngle);
        Angle discreteDelta = AngleMath.toDiscrete(continuousDelta);

        if (discreteDelta.in(Rotations) > 0.25) {
            velocity = velocity.unaryMinus();
            discreteDelta = discreteDelta.minus(Rotations.of(0.5));
        } else if (discreteDelta.in(Rotations) < -0.25) {
            velocity = velocity.unaryMinus();
            discreteDelta = discreteDelta.plus(Rotations.of(0.5));
        }

        Angle optimizedAngle = steerAngle.plus(discreteDelta);

        return new SwerveModuleState(velocity.in(MetersPerSecond), new Rotation2d(optimizedAngle));
    }

    public static SwerveModulePosition optimize(SwerveModulePosition target, Angle steerAngle) {
        Distance velocity = Meters.of(target.distanceMeters);
        
        Angle targetAngle = target.angle.getMeasure();
        Angle continuousDelta = targetAngle.minus(steerAngle);
        Angle discreteDelta = AngleMath.toDiscrete(continuousDelta);

        if (discreteDelta.in(Rotations) > 0.25) {
            velocity = velocity.unaryMinus();
            discreteDelta = discreteDelta.minus(Rotations.of(0.5));
        } else if (discreteDelta.in(Rotations) < -0.25) {
            velocity = velocity.unaryMinus();
            discreteDelta = discreteDelta.plus(Rotations.of(0.5));
        }

        Angle optimizedAngle = steerAngle.plus(discreteDelta);

        return new SwerveModulePosition(velocity.in(Meters), new Rotation2d(optimizedAngle));
    }

    public static SwerveModuleState cosineCorrect(SwerveModuleState target, Angle steerAngle) {
        return new SwerveModuleState(
            target.speedMetersPerSecond * getCosineError(target, steerAngle),
            target.angle
        );
    }

    public static double getCosineError(SwerveModuleState target, Angle steerAngle) {
        return Math.cos(target.angle.getMeasure().minus(steerAngle).in(Radians));
    }

    public static double getCosineError(SwerveModulePosition target, Angle steerAngle) {
        return Math.cos(target.angle.getMeasure().minus(steerAngle).in(Radians));
    }

    public static ChassisSpeeds sum(ChassisSpeeds a, ChassisSpeeds b) {
        return new ChassisSpeeds(
            a.vxMetersPerSecond + b.vxMetersPerSecond,
            a.vyMetersPerSecond + b.vyMetersPerSecond,
            a.omegaRadiansPerSecond + b.omegaRadiansPerSecond
        );
    }
}
