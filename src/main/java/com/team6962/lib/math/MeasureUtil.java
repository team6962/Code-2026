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

public class MeasureUtil {
    public static Angle toAngle(Measure<AngleUnit> measure) {
        return Radians.ofBaseUnits(measure.baseUnitMagnitude());
    }

    public static AngularVelocity toAngularVelocity(Measure<PerUnit<AngleUnit, TimeUnit>> measure) {
        return RadiansPerSecond.ofBaseUnits(measure.baseUnitMagnitude());
    }

    public static AngularAcceleration toAngularAcceleration(Measure<PerUnit<PerUnit<AngleUnit, TimeUnit>, TimeUnit>> measure) {
        double radians = measure.baseUnitMagnitude();
        return RadiansPerSecondPerSecond.ofBaseUnits(radians);
    }

    public static Distance toDistance(Measure<DistanceUnit> measure) {
        return Meters.ofBaseUnits(measure.baseUnitMagnitude());
    }

    public static LinearVelocity toLinearVelocity(Measure<PerUnit<DistanceUnit, TimeUnit>> measure) {
        return MetersPerSecond.ofBaseUnits(measure.baseUnitMagnitude());
    }

    public static LinearAcceleration toLinearAcceleration(Measure<PerUnit<PerUnit<DistanceUnit, TimeUnit>, TimeUnit>> measure) {
        return MetersPerSecondPerSecond.ofBaseUnits(measure.baseUnitMagnitude());
    }

    public static MomentOfInertia toMomentOfInertia(Measure<MultUnit<MultUnit<DistanceUnit, DistanceUnit>, MassUnit>> measure) {
        return KilogramSquareMeters.ofBaseUnits(measure.baseUnitMagnitude());
    }
}
