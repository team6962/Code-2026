package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.Test;

class SwerveKinematicsUtilTest {

  private static final double DELTA = 1e-9;

  @Test
  void optimize_AlignedModule_NoChange() {
    SwerveModuleState target = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModuleState result = SwerveKinematicsUtil.optimize(target, currentSteerAngle);

    assertEquals(1.0, result.speedMetersPerSecond, DELTA);
    assertEquals(0.0, result.angle.getDegrees(), DELTA);
  }

  @Test
  void optimize_SmallAngleDifference_NoInversion() {
    SwerveModuleState target = new SwerveModuleState(1.0, Rotation2d.fromDegrees(45));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModuleState result = SwerveKinematicsUtil.optimize(target, currentSteerAngle);

    assertEquals(1.0, result.speedMetersPerSecond, DELTA);
    assertEquals(45.0, result.angle.getDegrees(), DELTA);
  }

  @Test
  void optimize_LargeAngleDifference_InvertsVelocity() {
    SwerveModuleState target = new SwerveModuleState(1.0, Rotation2d.fromDegrees(180));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModuleState result = SwerveKinematicsUtil.optimize(target, currentSteerAngle);

    assertEquals(-1.0, result.speedMetersPerSecond, DELTA);
    assertEquals(0.0, result.angle.getDegrees(), DELTA);
  }

  @Test
  void optimize_135Degrees_InvertsAndFlips() {
    SwerveModuleState target = new SwerveModuleState(1.0, Rotation2d.fromDegrees(135));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModuleState result = SwerveKinematicsUtil.optimize(target, currentSteerAngle);

    assertEquals(-1.0, result.speedMetersPerSecond, DELTA);
    assertEquals(-45.0, result.angle.getDegrees(), DELTA);
  }

  @Test
  void optimize_Minus135Degrees_InvertsAndFlips() {
    SwerveModuleState target = new SwerveModuleState(1.0, Rotation2d.fromDegrees(-135));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModuleState result = SwerveKinematicsUtil.optimize(target, currentSteerAngle);

    assertEquals(-1.0, result.speedMetersPerSecond, DELTA);
    assertEquals(45.0, result.angle.getDegrees(), DELTA);
  }

  @Test
  void optimizeRelativePosition_SmallAngle_NoInversion() {
    SwerveModulePosition position = new SwerveModulePosition(1.0, Rotation2d.fromDegrees(30));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModulePosition result =
        SwerveKinematicsUtil.optimizeRelativePosition(position, currentSteerAngle);

    assertEquals(1.0, result.distanceMeters, DELTA);
    assertEquals(30.0, result.angle.getDegrees(), DELTA);
  }

  @Test
  void optimizeRelativePosition_LargeAngle_InvertsDistance() {
    SwerveModulePosition position = new SwerveModulePosition(1.0, Rotation2d.fromDegrees(180));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModulePosition result =
        SwerveKinematicsUtil.optimizeRelativePosition(position, currentSteerAngle);

    assertEquals(-1.0, result.distanceMeters, DELTA);
    assertEquals(0.0, result.angle.getDegrees(), DELTA);
  }

  @Test
  void optimizeAbsolutePosition_LargeAngle_DoesNotInvertDistance() {
    SwerveModulePosition position = new SwerveModulePosition(1.0, Rotation2d.fromDegrees(180));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModulePosition result =
        SwerveKinematicsUtil.optimizeAbsolutePosition(position, currentSteerAngle);

    assertEquals(1.0, result.distanceMeters, DELTA);
  }

  @Test
  void optimizeAbsolutePosition_WrapsAngle() {
    SwerveModulePosition position = new SwerveModulePosition(1.0, Rotation2d.fromDegrees(270));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModulePosition result =
        SwerveKinematicsUtil.optimizeAbsolutePosition(position, currentSteerAngle);

    assertEquals(1.0, result.distanceMeters, DELTA);
    assertEquals(-90.0, result.angle.getDegrees(), DELTA);
  }

  @Test
  void decreaseVelocityIfMisaligned_PerfectlyAligned_NoChange() {
    SwerveModuleState target = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModuleState result =
        SwerveKinematicsUtil.decreaseVelocityIfMisaligned(target, currentSteerAngle);

    assertEquals(1.0, result.speedMetersPerSecond, DELTA);
  }

  @Test
  void decreaseVelocityIfMisaligned_Perpendicular_ZeroVelocity() {
    SwerveModuleState target = new SwerveModuleState(1.0, Rotation2d.fromDegrees(90));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModuleState result =
        SwerveKinematicsUtil.decreaseVelocityIfMisaligned(target, currentSteerAngle);

    assertEquals(0.0, result.speedMetersPerSecond, DELTA);
  }

  @Test
  void decreaseVelocityIfMisaligned_45Degrees_ReducedVelocity() {
    SwerveModuleState target = new SwerveModuleState(1.0, Rotation2d.fromDegrees(45));
    Angle currentSteerAngle = Degrees.of(0);

    SwerveModuleState result =
        SwerveKinematicsUtil.decreaseVelocityIfMisaligned(target, currentSteerAngle);

    assertEquals(Math.cos(Math.toRadians(45)), result.speedMetersPerSecond, DELTA);
  }

  @Test
  void getCosineOfSteerError_State_Aligned_ReturnsOne() {
    SwerveModuleState target = new SwerveModuleState(1.0, Rotation2d.fromDegrees(0));
    Angle currentSteerAngle = Degrees.of(0);

    double result = SwerveKinematicsUtil.getCosineOfSteerError(target, currentSteerAngle);

    assertEquals(1.0, result, DELTA);
  }

  @Test
  void getCosineOfSteerError_State_Perpendicular_ReturnsZero() {
    SwerveModuleState target = new SwerveModuleState(1.0, Rotation2d.fromDegrees(90));
    Angle currentSteerAngle = Degrees.of(0);

    double result = SwerveKinematicsUtil.getCosineOfSteerError(target, currentSteerAngle);

    assertEquals(0.0, result, DELTA);
  }

  @Test
  void getCosineOfSteerError_Position_Aligned_ReturnsOne() {
    SwerveModulePosition target = new SwerveModulePosition(1.0, Rotation2d.fromDegrees(0));
    Angle currentSteerAngle = Degrees.of(0);

    double result = SwerveKinematicsUtil.getCosineOfSteerError(target, currentSteerAngle);

    assertEquals(1.0, result, DELTA);
  }

  @Test
  void getCosineOfSteerError_Position_Perpendicular_ReturnsZero() {
    SwerveModulePosition target = new SwerveModulePosition(1.0, Rotation2d.fromDegrees(90));
    Angle currentSteerAngle = Degrees.of(0);

    double result = SwerveKinematicsUtil.getCosineOfSteerError(target, currentSteerAngle);

    assertEquals(0.0, result, DELTA);
  }

  @Test
  void addChassisSpeeds_BothZero_ReturnsZero() {
    ChassisSpeeds a = new ChassisSpeeds(0, 0, 0);
    ChassisSpeeds b = new ChassisSpeeds(0, 0, 0);

    ChassisSpeeds result = SwerveKinematicsUtil.addChassisSpeeds(a, b);

    assertEquals(0.0, result.vxMetersPerSecond, DELTA);
    assertEquals(0.0, result.vyMetersPerSecond, DELTA);
    assertEquals(0.0, result.omegaRadiansPerSecond, DELTA);
  }

  @Test
  void addChassisSpeeds_AddsComponents() {
    ChassisSpeeds a = new ChassisSpeeds(1.0, 2.0, 0.5);
    ChassisSpeeds b = new ChassisSpeeds(3.0, 4.0, 1.5);

    ChassisSpeeds result = SwerveKinematicsUtil.addChassisSpeeds(a, b);

    assertEquals(4.0, result.vxMetersPerSecond, DELTA);
    assertEquals(6.0, result.vyMetersPerSecond, DELTA);
    assertEquals(2.0, result.omegaRadiansPerSecond, DELTA);
  }

  @Test
  void addChassisSpeeds_NegativeValues() {
    ChassisSpeeds a = new ChassisSpeeds(1.0, -2.0, 0.5);
    ChassisSpeeds b = new ChassisSpeeds(-3.0, 4.0, -1.5);

    ChassisSpeeds result = SwerveKinematicsUtil.addChassisSpeeds(a, b);

    assertEquals(-2.0, result.vxMetersPerSecond, DELTA);
    assertEquals(2.0, result.vyMetersPerSecond, DELTA);
    assertEquals(-1.0, result.omegaRadiansPerSecond, DELTA);
  }
}
