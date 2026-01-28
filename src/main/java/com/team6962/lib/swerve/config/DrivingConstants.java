package com.team6962.lib.swerve.config;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Configuration constants for swerve drive motion control, including velocity and acceleration
 * limits, as well as feedback controller gains for translational and angular motion.
 */
public class DrivingConstants {
  /**
   * The maximum linear velocity when in precise driving mode, which provides finer control at lower
   * speeds.
   */
  public LinearVelocity PreciseDriveVelocity;

  /** The maximum linear acceleration when in precise driving mode. */
  public LinearAcceleration PreciseDriveAcceleration;

  /** The proportional gain (kP) for the translation feedback controller. */
  public double TranslationFeedbackKP;

  /** The integral gain (kI) for the translation feedback controller. */
  public double TranslationFeedbackKI;

  /** The derivative gain (kD) for the translation feedback controller. */
  public double TranslationFeedbackKD;

  /** The proportional gain (kP) for the angle (heading) feedback controller. */
  public double AngleFeedbackKP;

  /** The integral gain (kI) for the angle (heading) feedback controller. */
  public double AngleFeedbackKI;

  /** The derivative gain (kD) for the angle (heading) feedback controller. */
  public double AngleFeedbackKD;

  /** The maximum linear velocity the swerve drive can achieve. */
  public LinearVelocity MaxLinearVelocity;

  /** The maximum linear acceleration the swerve drive can achieve. */
  public LinearAcceleration MaxLinearAcceleration;

  /** The maximum angular velocity (rotation rate) the swerve drive can achieve. */
  public AngularVelocity MaxAngularVelocity;

  /** The maximum angular acceleration the swerve drive can achieve. */
  public AngularAcceleration MaxAngularAcceleration;

  /**
   * Sets the precise drive velocity.
   *
   * @param velocity The maximum linear velocity in precise driving mode
   * @return This instance for method chaining
   */
  public DrivingConstants withPreciseDriveVelocity(LinearVelocity velocity) {
    this.PreciseDriveVelocity = velocity;
    return this;
  }

  /**
   * Sets the precise drive acceleration.
   *
   * @param acceleration The maximum linear acceleration in precise driving mode
   * @return This instance for method chaining
   */
  public DrivingConstants withPreciseDriveAcceleration(LinearAcceleration acceleration) {
    this.PreciseDriveAcceleration = acceleration;
    return this;
  }

  /**
   * Sets the proportional gain for the translation feedback controller.
   *
   * @param kp The proportional gain value
   * @return This instance for method chaining
   */
  public DrivingConstants withTranslationFeedbackKP(double kp) {
    this.TranslationFeedbackKP = kp;
    return this;
  }

  /**
   * Sets the integral gain for the translation feedback controller.
   *
   * @param ki The integral gain value
   * @return This instance for method chaining
   */
  public DrivingConstants withTranslationFeedbackKI(double ki) {
    this.TranslationFeedbackKI = ki;
    return this;
  }

  /**
   * Sets the derivative gain for the translation feedback controller.
   *
   * @param kd The derivative gain value
   * @return This instance for method chaining
   */
  public DrivingConstants withTranslationFeedbackKD(double kd) {
    this.TranslationFeedbackKD = kd;
    return this;
  }

  /**
   * Sets the proportional gain for the angle feedback controller.
   *
   * @param kp The proportional gain value
   * @return This instance for method chaining
   */
  public DrivingConstants withAngleFeedbackKP(double kp) {
    this.AngleFeedbackKP = kp;
    return this;
  }

  /**
   * Sets the integral gain for the angle feedback controller.
   *
   * @param ki The integral gain value
   * @return This instance for method chaining
   */
  public DrivingConstants withAngleFeedbackKI(double ki) {
    this.AngleFeedbackKI = ki;
    return this;
  }

  /**
   * Sets the derivative gain for the angle feedback controller.
   *
   * @param kd The derivative gain value
   * @return This instance for method chaining
   */
  public DrivingConstants withAngleFeedbackKD(double kd) {
    this.AngleFeedbackKD = kd;
    return this;
  }

  /**
   * Sets the maximum linear velocity.
   *
   * @param velocity The maximum linear velocity
   * @return This instance for method chaining
   */
  public DrivingConstants withMaxLinearVelocity(LinearVelocity velocity) {
    this.MaxLinearVelocity = velocity;
    return this;
  }

  /**
   * Sets the maximum linear acceleration.
   *
   * @param acceleration The maximum linear acceleration
   * @return This instance for method chaining
   */
  public DrivingConstants withMaxLinearAcceleration(LinearAcceleration acceleration) {
    this.MaxLinearAcceleration = acceleration;
    return this;
  }

  /**
   * Sets the maximum angular velocity.
   *
   * @param velocity The maximum angular velocity
   * @return This instance for method chaining
   */
  public DrivingConstants withMaxAngularVelocity(AngularVelocity velocity) {
    this.MaxAngularVelocity = velocity;
    return this;
  }

  /**
   * Sets the maximum angular acceleration.
   *
   * @param acceleration The maximum angular acceleration
   * @return This instance for method chaining
   */
  public DrivingConstants withMaxAngularAcceleration(AngularAcceleration acceleration) {
    this.MaxAngularAcceleration = acceleration;
    return this;
  }

  /**
   * Creates a {@link TrapezoidProfile.Constraints} object for translational motion based on the
   * configured maximum linear velocity and acceleration.
   *
   * @return The translation constraints for trajectory generation
   */
  public TrapezoidProfile.Constraints getTranslationConstraints() {
    return new TrapezoidProfile.Constraints(
        MaxLinearVelocity.in(MetersPerSecond), MaxLinearAcceleration.in(MetersPerSecondPerSecond));
  }

  /**
   * Creates a {@link TrapezoidProfile.Constraints} object for rotational motion based on the
   * configured maximum angular velocity and acceleration.
   *
   * @return The rotation constraints for trajectory generation
   */
  public TrapezoidProfile.Constraints getRotationConstraints() {
    return new TrapezoidProfile.Constraints(
        MaxAngularVelocity.in(RadiansPerSecond),
        MaxAngularAcceleration.in(RadiansPerSecondPerSecond));
  }
}
