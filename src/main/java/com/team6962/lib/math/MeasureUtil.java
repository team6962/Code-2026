package com.team6962.lib.math;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.MassUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

/**
 * Utility class for working with Measure objects.
 */
public class MeasureUtil {
    /**
     * Converts a {@code Measure<AngleUnit>} to an equivalent {@link Angle} object.
     * 
     * @param measure The {@code Measure<AngleUnit>} to convert
     * @return The equivalent {@link Angle} object
     */
    public static Angle toAngle(Measure<AngleUnit> measure) {
        return Radians.ofBaseUnits(measure.baseUnitMagnitude());
    }

    /**
     * Converts a {@code Measure<PerUnit<AngleUnit, TimeUnit>>} to an equivalent
     * {@link AngularVelocity}.
     * 
     * @param measure The {@code Measure<PerUnit<AngleUnit, TimeUnit>>} to convert
     * @returnThe The equivalent {@link AngularVelocity} object
     */
    public static AngularVelocity toAngularVelocity(Measure<PerUnit<AngleUnit, TimeUnit>> measure) {
        return RadiansPerSecond.ofBaseUnits(measure.baseUnitMagnitude());
    }

    /**
     * Converts a {@code Measure<PerUnit<PerUnit<AngleUnit, TimeUnit>, TimeUnit>>}
     * to an equivalent {@link AngularAcceleration}.
     * 
     * @param measure The {@code Measure<PerUnit<PerUnit<AngleUnit, TimeUnit>, TimeUnit>>}
     *                to convert
     * @return The equivalent {@link AngularAcceleration} object
     */
    public static AngularAcceleration toAngularAcceleration(Measure<PerUnit<PerUnit<AngleUnit, TimeUnit>, TimeUnit>> measure) {
        double radians = measure.baseUnitMagnitude();
        return RadiansPerSecondPerSecond.ofBaseUnits(radians);
    }

    /**
     * Converts a {@code Measure<DistanceUnit>} to an equivalent
     * {@link Distance}.
     * 
     * @param measure The {@code Measure<DistanceUnit>} to convert
     * @return The equivalent {@link Distance} object
     */
    public static Distance toDistance(Measure<DistanceUnit> measure) {
        return Meters.ofBaseUnits(measure.baseUnitMagnitude());
    }

    /**
     * Converts a {@code Measure<PerUnit<DistanceUnit, TimeUnit>>} to an
     * equivalent {@link LinearVelocity}.
     * 
     * @param measure The {@code Measure<PerUnit<DistanceUnit, TimeUnit>>} to convert
     * @return The equivalent {@link LinearVelocity} object
     */
    public static LinearVelocity toLinearVelocity(Measure<PerUnit<DistanceUnit, TimeUnit>> measure) {
        return MetersPerSecond.ofBaseUnits(measure.baseUnitMagnitude());
    }

    /**
     * Converts a {@code Measure<PerUnit<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>>}
     * to an equivalent {@link LinearAcceleration}.
     * 
     * @param measure The {@code Measure<PerUnit<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>>}
     *                to convert
     * @return The equivalent {@link LinearAcceleration} object
     */
    public static LinearAcceleration toLinearAcceleration(Measure<PerUnit<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>> measure) {
        return MetersPerSecondPerSecond.ofBaseUnits(measure.baseUnitMagnitude());
    }

    /**
     * Converts a {@code Measure<MultUnit<MultUnit<DistanceUnit, DistanceUnit>, MassUnit>>}
     * to an equivalent {@link MomentOfInertia}.
     * 
     * @param measure The {@code Measure<MultUnit<MultUnit<DistanceUnit, DistanceUnit>, MassUnit>>}
     *                to convert
     * @return The equivalent {@link MomentOfInertia} object
     */
    public static MomentOfInertia toMomentOfInertia(Measure<MultUnit<MultUnit<DistanceUnit, DistanceUnit>, MassUnit>> measure) {
        return KilogramSquareMeters.ofBaseUnits(measure.baseUnitMagnitude());
    }
}
