package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class WheelMath {
    public static Distance toLinear(Angle angle, Distance radius) {
        return Meters.of(angle.in(Radians) * radius.in(Meters));
    }

    public static Angle toAngular(Distance arcLength, Distance radius) {
        return Radians.of(arcLength.in(Meters) / radius.in(Meters));
    }

    public static LinearVelocity toLinear(AngularVelocity angularVelocity, Distance radius) {
        return MetersPerSecond.of(angularVelocity.in(RadiansPerSecond) * radius.in(Meters));
    }

    public static AngularVelocity toAngular(LinearVelocity linearVelocity, Distance radius) {
        return RadiansPerSecond.of(linearVelocity.in(MetersPerSecond) / radius.in(Meters));
    }

    public static LinearAcceleration toLinear(AngularAcceleration angularAcceleration, Distance radius) {
        return MetersPerSecondPerSecond.of(angularAcceleration.in(RadiansPerSecondPerSecond) * radius.in(Meters));
    }

    public static AngularAcceleration toAngular(LinearAcceleration linearAcceleration, Distance radius) {
        return RadiansPerSecondPerSecond.of(linearAcceleration.in(MetersPerSecondPerSecond) / radius.in(Meters));
    }
}
