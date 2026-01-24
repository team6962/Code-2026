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

/** Utility methods for swerve kinematics calculations. */
public class SwerveKinematicsUtil {
  /**
   * Optimizes a target swerve module state to minimize the rotation needed for the steering motor
   * to reach its target angle. The angle of the returned state will always be within ±90° of the
   * current steer angle, and it will result in the same robot movement once the state is reached
   * (though the optimized target state will often be reached faster than the unoptimized state). In
   * some cases, the drive velocity of the returned state may be negated to achieve this
   * optimization.
   *
   * @param target The target swerve module state, which is not modified
   * @param steerAngle The current angle of the steering motor
   * @return The optimized swerve module state
   */
  public static SwerveModuleState optimize(SwerveModuleState target, Angle steerAngle) {
    // Get the target velocity and angle as measures
    LinearVelocity targetVelocity = MetersPerSecond.of(target.speedMetersPerSecond);
    Angle targetAngle = target.angle.getMeasure();

    // Calculate the difference between the target angle and the current
    // steer angle, wrapped at ±180° to avoid rotating the wheel the long
    // way around
    Angle angularMovement = AngleMath.toDiscrete(targetAngle.minus(steerAngle));

    // If the angle difference is greater than 90°, invert the drive
    // velocity and flip the steer target angle by 180°
    if (angularMovement.in(Rotations) > 0.25) {
      targetVelocity = targetVelocity.unaryMinus();
      angularMovement = angularMovement.minus(Rotations.of(0.5));
    } else if (angularMovement.in(Rotations) < -0.25) {
      targetVelocity = targetVelocity.unaryMinus();
      angularMovement = angularMovement.plus(Rotations.of(0.5));
    }

    // Calculate the optimized steer angle and return the optimized state
    Angle optimizedTargetAngle = steerAngle.plus(angularMovement);

    return new SwerveModuleState(
        targetVelocity.in(MetersPerSecond), new Rotation2d(optimizedTargetAngle));
  }

  /**
   * Optimizes a target swerve module position to minimize the rotation needed for the steering
   * motor to reach its target angle. The angle of the returned movement will always be within ±90°
   * of the current steer angle, and it will result in the same robot movement once the movement is
   * reached (though the optimized target movement will often be reached faster than the unoptimized
   * movement). In some cases, the distance of the returned movement may be negated to achieve this
   * optimization.
   *
   * <p>Note that this method expects the swerve module position's distance field to represent a
   * change in position (not an absolute position), where {@code 0} means no movement. This is
   * different from the {@link #optimizeAbsolutePosition optimizeAbsolutePosition()} method, which
   * assumes that the distance field represents an absolute position that cannot be negated.
   *
   * @param position The target swerve module position, which is not modified
   * @param steerAngle The current angle of the steering motor
   * @return The optimized swerve module position, with a possibly negated distance field
   */
  public static SwerveModulePosition optimizeRelativePosition(
      SwerveModulePosition position, Angle steerAngle) {
    // Get the target distance to travel and angle as measures
    Distance distance = Meters.of(position.distanceMeters);
    Angle targetAngle = position.angle.getMeasure();

    // Calculate the difference between the target angle and the current
    // steer angle, wrapped at ±180° to avoid rotating the wheel the long
    // way around
    Angle angularMovement = AngleMath.toDiscrete(targetAngle.minus(steerAngle));

    // If the angle difference is greater than 90°, invert the drive
    // distance and flip the steer target angle by 180°
    if (angularMovement.in(Rotations) > 0.25) {
      distance = distance.unaryMinus();
      angularMovement = angularMovement.minus(Rotations.of(0.5));
    } else if (angularMovement.in(Rotations) < -0.25) {
      distance = distance.unaryMinus();
      angularMovement = angularMovement.plus(Rotations.of(0.5));
    }

    // Calculate the optimized steer angle and return the optimized swerve
    // module position
    Angle optimizedAngle = steerAngle.plus(angularMovement);

    return new SwerveModulePosition(distance.in(Meters), new Rotation2d(optimizedAngle));
  }

  /**
   * Optimizes a target swerve module position to minimize the rotation needed for the steering
   * motor to reach its target angle. The angle of the returned position will always be within ±180°
   * of the current steer angle, and it will result in the same robot movement once the position is
   * reached.
   *
   * <p>Note that this optimizeAbsolutePosition method assumes that the swerve module position's
   * distance field represents an absolute position, which should not be negated. If you have a
   * relative movement that needs to be optimized, use the {@link #optimizeRelativePosition
   * optimizeRelativePosition()} method instead to minimize steering rotation.
   *
   * @param position The target swerve module position, which is not modified
   * @param steerAngle The current angle of the steering motor
   * @return The optimized swerve module position
   */
  public static SwerveModulePosition optimizeAbsolutePosition(
      SwerveModulePosition position, Angle steerAngle) {
    // Get the target distance to travel and angle as measures
    Distance distance = Meters.of(position.distanceMeters);
    Angle targetAngle = position.angle.getMeasure();

    // Calculate the difference between the target angle and the current
    // steer angle, wrapped at ±180° to avoid rotating the wheel the long
    // way around
    Angle angularMovement = AngleMath.toDiscrete(targetAngle.minus(steerAngle));

    // Calculate the optimized steer angle and return the optimized swerve
    // module movement
    Angle optimizedAngle = steerAngle.plus(angularMovement);

    return new SwerveModulePosition(distance.in(Meters), new Rotation2d(optimizedAngle));
  }

  /**
   * Decreases the velocity of a target swerve module state based on how misaligned the module is
   * from its target angle. The more misaligned the module is, the more the velocity is decreased.
   * If the module is perfectly aligned, the velocity is unchanged, and if the module is
   * perpendicularly misaligned (±90°), the velocity is reduced to zero.
   *
   * @param target The target swerve module state
   * @param steerAngle The current angle of the steering motor
   * @return The swerve module state with a possibly decreased velocity
   */
  public static SwerveModuleState decreaseVelocityIfMisaligned(
      SwerveModuleState target, Angle steerAngle) {
    return new SwerveModuleState(
        target.speedMetersPerSecond * getCosineOfSteerError(target, steerAngle), target.angle);
  }

  /**
   * Returns the cosine of the steering error between the target swerve module state and the current
   * steer angle. When the module is perfectly aligned, this value will be 1, and when the module is
   * perpendicularly misaligned (±90°), this value will be 0. This value is used by {@link
   * #decreaseVelocityIfMisaligned decreaseVelocityIfMisaligned()} to decrease the drive velocity
   * based on how misaligned the module is from its target angle.
   *
   * @param target The target swerve module state
   * @param steerAngle The current angle of the steering motor
   * @return The cosine of the steering error
   */
  public static double getCosineOfSteerError(SwerveModuleState target, Angle steerAngle) {
    return Math.cos(target.angle.getMeasure().minus(steerAngle).in(Radians));
  }

  /**
   * Returns the cosine of the steering error between the target swerve module position and the
   * current steer angle. When the module is perfectly aligned, this value will be 1, and when the
   * module is perpendicularly misaligned (±90°), this value will be 0. This value is can be used to
   * decrease the drive velocity based on how misaligned the module is from its target angle.
   *
   * @param target The target swerve module position
   * @param steerAngle The current angle of the steering motor
   * @return The cosine of the steering error
   */
  public static double getCosineOfSteerError(SwerveModulePosition target, Angle steerAngle) {
    return Math.cos(target.angle.getMeasure().minus(steerAngle).in(Radians));
  }

  /**
   * Sums two ChassisSpeeds objects together component-wise.
   *
   * @param a A ChassisSpeeds object
   * @param b Another ChassisSpeeds object
   * @return The sum of the two ChassisSpeeds objects
   */
  public static ChassisSpeeds addChassisSpeeds(ChassisSpeeds a, ChassisSpeeds b) {
    return new ChassisSpeeds(
        a.vxMetersPerSecond + b.vxMetersPerSecond,
        a.vyMetersPerSecond + b.vyMetersPerSecond,
        a.omegaRadiansPerSecond + b.omegaRadiansPerSecond);
  }
}
