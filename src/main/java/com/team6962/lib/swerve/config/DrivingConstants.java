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

public class DrivingConstants {
    public LinearVelocity PreciseDriveVelocity;
    public LinearAcceleration PreciseDriveAcceleration;

    public double TranslationFeedbackKP;
    public double TranslationFeedbackKI;
    public double TranslationFeedbackKD;

    public double AngleFeedbackKP;
    public double AngleFeedbackKI;
    public double AngleFeedbackKD;

    public LinearVelocity MaxLinearVelocity;
    public LinearAcceleration MaxLinearAcceleration;

    public AngularVelocity MaxAngularVelocity;
    public AngularAcceleration MaxAngularAcceleration;

    public DrivingConstants withPreciseDriveVelocity(LinearVelocity velocity) {
        this.PreciseDriveVelocity = velocity;
        return this;
    }

    public DrivingConstants withPreciseDriveAcceleration(LinearAcceleration acceleration) {
        this.PreciseDriveAcceleration = acceleration;
        return this;
    }

    public DrivingConstants withTranslationFeedbackKP(double kp) {
        this.TranslationFeedbackKP = kp;
        return this;
    }

    public DrivingConstants withTranslationFeedbackKI(double ki) {
        this.TranslationFeedbackKI = ki;
        return this;
    }

    public DrivingConstants withTranslationFeedbackKD(double kd) {
        this.TranslationFeedbackKD = kd;
        return this;
    }

    public DrivingConstants withAngleFeedbackKP(double kp) {
        this.AngleFeedbackKP = kp;
        return this;
    }

    public DrivingConstants withAngleFeedbackKI(double ki) {
        this.AngleFeedbackKI = ki;
        return this;
    }

    public DrivingConstants withAngleFeedbackKD(double kd) {
        this.AngleFeedbackKD = kd;
        return this;
    }

    public DrivingConstants withMaxLinearVelocity(LinearVelocity velocity) {
        this.MaxLinearVelocity = velocity;
        return this;
    }

    public DrivingConstants withMaxLinearAcceleration(LinearAcceleration acceleration) {
        this.MaxLinearAcceleration = acceleration;
        return this;
    }

    public DrivingConstants withMaxAngularVelocity(AngularVelocity velocity) {
        this.MaxAngularVelocity = velocity;
        return this;
    }

    public DrivingConstants withMaxAngularAcceleration(AngularAcceleration acceleration) {
        this.MaxAngularAcceleration = acceleration;
        return this;
    }

    public TrapezoidProfile.Constraints getTranslationConstraints() {
        return new TrapezoidProfile.Constraints(
            MaxLinearVelocity.in(MetersPerSecond),
            MaxLinearAcceleration.in(MetersPerSecondPerSecond)
        );
    }

    public TrapezoidProfile.Constraints getRotationConstraints() {
        return new TrapezoidProfile.Constraints(
            MaxAngularVelocity.in(RadiansPerSecond),
            MaxAngularAcceleration.in(RadiansPerSecondPerSecond)
        );
    }
}
