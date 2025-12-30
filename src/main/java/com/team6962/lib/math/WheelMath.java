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

/**
 * Utility class for converting between wheel angular measurements and linear
 * measurements at the edge of the wheel.
 */
public class WheelMath {
    /**
     * Converts an angle that a wheel has rotated to the linear distance that
     * the edge of the wheel travels.
     * 
     * @param angle The angle the wheel is rotated
     * @param radius The radius of the wheel
     * @return The linear distance traveled at the edge of the wheel
     */
    public static Distance toLinear(Angle angle, Distance radius) {
        return Meters.of(angle.in(Radians) * radius.in(Meters));
    }

    /**
     * Converts a linear distance traveled at the edge of a wheel to the angle
     * that the wheel has rotated.
     * 
     * @param arcLength The linear distance traveled at the edge of the wheel
     * @param radius The radius of the wheel
     * @return The angle the wheel has rotated
     */
    public static Angle toAngular(Distance arcLength, Distance radius) {
        return Radians.of(arcLength.in(Meters) / radius.in(Meters));
    }

    /**
     * Converts an angular velocity of a wheel to the linear velocity at the
     * edge of the wheel.
     * 
     * @param angularVelocity The angular velocity of the wheel
     * @param radius The radius of the wheel
     * @return The linear velocity at the edge of the wheel
     */
    public static LinearVelocity toLinear(AngularVelocity angularVelocity, Distance radius) {
        return MetersPerSecond.of(angularVelocity.in(RadiansPerSecond) * radius.in(Meters));
    }

    /**
     * Converts a linear velocity at the edge of a wheel to the angular
     * velocity of the wheel.
     * 
     * @param linearVelocity The linear velocity at the edge of the wheel
     * @param radius The radius of the wheel
     * @return The angular velocity of the wheel
     */
    public static AngularVelocity toAngular(LinearVelocity linearVelocity, Distance radius) {
        return RadiansPerSecond.of(linearVelocity.in(MetersPerSecond) / radius.in(Meters));
    }

    /**
     * Converts an angular acceleration of a wheel to the linear acceleration
     * at the edge of the wheel.
     * 
     * @param angularAcceleration The angular acceleration of the wheel
     * @param radius The radius of the wheel
     * @return The linear acceleration at the edge of the wheel
     */
    public static LinearAcceleration toLinear(AngularAcceleration angularAcceleration, Distance radius) {
        return MetersPerSecondPerSecond.of(angularAcceleration.in(RadiansPerSecondPerSecond) * radius.in(Meters));
    }

    /**
     * Converts a linear acceleration at the edge of a wheel to the angular
     * acceleration of the wheel.
     * 
     * @param linearAcceleration The linear acceleration at the edge of the wheel
     * @param radius The radius of the wheel
     * @return The angular acceleration of the wheel
     */
    public static AngularAcceleration toAngular(LinearAcceleration linearAcceleration, Distance radius) {
        return RadiansPerSecondPerSecond.of(linearAcceleration.in(MetersPerSecondPerSecond) / radius.in(Meters));
    }
}
