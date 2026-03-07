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
public class DrivingConstants implements Cloneable {
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

  /** The maximum linear velocity used for autonomous. */
  public LinearVelocity AutoLinearVelocity;

  /** The maximum linear acceleration used for autonomous. */
  public LinearAcceleration AutoLinearAcceleration;

  /** The maximum angular velocity used for autonomous. */
  public AngularVelocity AutoAngularVelocity;

  /** The maximum angular acceleration used for autonomous. */
  public AngularAcceleration AutoAngularAcceleration;

  /**
   * The autonomous driving attempts to maintain a translational velocity equal to the translation
   * velocity in the motion profile plus the translational acceleration multiplied by this scalar.
   * This helps the drivetrain stay on the motion profile when accelerating and decelerating.
   */
  public double AutoLinearAccelerationScalar;

  /**
   * The autonomous driving attempts to maintain an angular velocity equal to the angular velocity
   * in the motion profile plus the angular acceleration multiplied by this scalar. This helps the
   * drivetrain stay on the motion profile when accelerating and decelerating rotationally.
   */
  public double AutoAngularAccelerationScalar;

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
   * Sets the maximum linear velocity for autonomous mode.
   *
   * @param velocity The maximum linear velocity for autonomous mode
   * @return This instance for method chaining
   */
  public DrivingConstants withAutoLinearVelocity(LinearVelocity velocity) {
    this.AutoLinearVelocity = velocity;
    return this;
  }

  /**
   * Sets the maximum linear acceleration for autonomous mode.
   *
   * @param acceleration The maximum linear acceleration for autonomous mode
   * @return This instance for method chaining
   */
  public DrivingConstants withAutoLinearAcceleration(LinearAcceleration acceleration) {
    this.AutoLinearAcceleration = acceleration;
    return this;
  }

  /**
   * Sets the maximum angular velocity for autonomous mode.
   *
   * @param velocity The maximum angular velocity for autonomous mode
   * @return This instance for method chaining
   */
  public DrivingConstants withAutoAngularVelocity(AngularVelocity velocity) {
    this.AutoAngularVelocity = velocity;
    return this;
  }

  /**
   * Sets the maximum angular acceleration for autonomous mode.
   *
   * @param acceleration The maximum angular acceleration for autonomous mode
   * @return This instance for method chaining
   */
  public DrivingConstants withAutoAngularAcceleration(AngularAcceleration acceleration) {
    this.AutoAngularAcceleration = acceleration;
    return this;
  }

  /**
   * Sets the auto linear acceleration scalar.
   *
   * @param scalar The scalar to multiply the linear acceleration by when calculating target
   *     velocity
   * @return This instance for method chaining
   */
  public DrivingConstants withAutoLinearAccelerationScalar(double scalar) {
    this.AutoLinearAccelerationScalar = scalar;
    return this;
  }

  /**
   * Sets the auto angular acceleration scalar.
   *
   * @param scalar The scalar to multiply the angular acceleration by when calculating target
   *     velocity
   * @return This instance for method chaining
   */
  public DrivingConstants withAutoAngularAccelerationScalar(double scalar) {
    this.AutoAngularAccelerationScalar = scalar;
    return this;
  }

  /**
   * Creates a {@link TrapezoidProfile.Constraints} object for translational motion based on the
   * configured maximum linear velocity and acceleration for autonomous mode.
   *
   * @return The translation constraints for trajectory generation
   */
  public TrapezoidProfile.Constraints getTranslationConstraints() {
    return new TrapezoidProfile.Constraints(
        AutoLinearVelocity.in(MetersPerSecond),
        AutoLinearAcceleration.in(MetersPerSecondPerSecond));
  }

  /**
   * Creates a {@link TrapezoidProfile.Constraints} object for rotational motion based on the
   * configured maximum angular velocity and acceleration for autonomous mode.
   *
   * @return The rotation constraints for trajectory generation
   */
  public TrapezoidProfile.Constraints getRotationConstraints() {
    return new TrapezoidProfile.Constraints(
        AutoAngularVelocity.in(RadiansPerSecond),
        AutoAngularAcceleration.in(RadiansPerSecondPerSecond));
  }

  @Override
  public DrivingConstants clone() {
    try {
      return (DrivingConstants) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone should be supported", e);
    }
  }
}
