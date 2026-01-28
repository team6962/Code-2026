package com.team6962.lib.math;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

/** Utility methods for working with angles. */
public class AngleMath {
  /**
   * Converts a continuous angle (which can go past 360° to 361° and beyond) to a discrete angle,
   * which is wrapped around to restrict it to the range -180° to 180°.
   *
   * @param continuousAngle The continuous angle to convert
   * @return The equivalent discrete angle, wrapped to the range -180° to 180°
   */
  public static Angle toDiscrete(Angle continuousAngle) {
    // Gets the angle in radians
    double continuousRadians = continuousAngle.in(Radians);

    // Finds a discrete angle between 0 and 2π
    double discreteRadians =
        continuousRadians - Math.floor(continuousRadians / (2 * Math.PI)) * (2 * Math.PI);

    // Maps angles greater than π to negative angles, so the output range
    // is -π to π instead of 0 to 2π
    if (discreteRadians > Math.PI) {
      discreteRadians -= 2 * Math.PI;
    }

    return Radians.of(discreteRadians);
  }

  /**
   * Converts a discrete angle (which is wrapped to the range -180° to 180°) to a continuous angle,
   * which can go past 360° to 361° and beyond. Given a discrete angle, there are infinite possible
   * equivalent continuous angles, which are each 360° apart, so this method requires a nearby
   * continuous angle to determine which continuous angle to return. The returned continuous angle
   * will be the one that is closest to the provided nearby continuous angle.
   *
   * @param discreteAngle The discrete angle to convert
   * @param nearbyContinuousAngle A nearby continuous angle used to determine which possible output
   *     to return
   * @return The equivalent continuous angle closest to the provided nearby continuous angle
   */
  public static Angle toContinuous(Angle discreteAngle, Angle nearbyContinuousAngle) {
    double discreteRadians = discreteAngle.in(Radians);
    double nearbyRadians = nearbyContinuousAngle.in(Radians);
    double nearbyMultipleOfTwoPi = Math.round(nearbyRadians / (2 * Math.PI)) * (2 * Math.PI);
    double continuousRadians = discreteRadians + nearbyMultipleOfTwoPi;

    return Radians.of(continuousRadians);
  }
}
