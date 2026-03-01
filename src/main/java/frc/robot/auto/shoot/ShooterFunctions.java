package frc.robot.auto.shoot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.io.IOException;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;

import com.team6962.lib.math.CSVLoader;
import com.team6962.lib.math.MeasureUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * A set of functions that can be used to calculate shooting parameters (hood angle,
 * flywheel velocity, etc) based on the current state of the robot and the target.
 *
 * These functions are generated from a CSV file containing empirical data of the
 * shooter's performance at various distances and angles. The CSV file should have
 * the following columns: distance (in inches), hood angle (in degrees), flywheel
 * velocity (in rotations per second), and flight time (in seconds). The class uses
 * spline interpolation to create continuous functions from the discrete data points
 * in the CSV file.
 */
public class ShooterFunctions {
  /** The minimum distance for which the shooter functions are valid. */
  private Distance minDistance;
  /** The maximum distance for which the shooter functions are valid. */
  private Distance maxDistance;

  /** The minimum hood angle for which the shooter functions are valid. */
  private Angle minHoodAngle;
  /** The maximum hood angle for which the shooter functions are valid. */
  private Angle maxHoodAngle;

  /** The function that maps distance to hood angle. */
  private UnivariateFunction hoodAngleFunction;
  /** The function that maps distance to flywheel velocity. */
  private UnivariateFunction flywheelVelocityFunction;
  /** The function that maps hood angle to distance. */
  private UnivariateFunction distanceFunction;
  /** The function that maps distance to flight time. */
  private UnivariateFunction flightTimeFunction;

  public ShooterFunctions(String filePath) {
    try {
      SplineInterpolator interpolator = new SplineInterpolator();
      double[][] data = CSVLoader.loadCSV(filePath);

      double[] distances = new double[data.length];
      double[] hoodAngles = new double[data.length];
      double[] flywheelVelocities = new double[data.length];
      double[] flightTimes = new double[data.length];

      double minDistanceInches = Double.MAX_VALUE;
      double maxDistanceInches = Double.MIN_VALUE;
      double minHoodAngleDegrees = Double.MAX_VALUE;
      double maxHoodAngleDegrees = Double.MIN_VALUE;

      for (int i = 0; i < data.length; i++) {
        distances[i] = data[i][0];
        hoodAngles[i] = data[i][1];
        flywheelVelocities[i] = data[i][2];
        flightTimes[i] = data[i][3];

        double distance = data[i][0];
        double hoodAngle = data[i][1];

        if (distance < minDistanceInches) {
          minDistanceInches = distance;
        }

        if (distance > maxDistanceInches) {
          maxDistanceInches = distance;
        }

        if (hoodAngle < minHoodAngleDegrees) {
          minHoodAngleDegrees = hoodAngle;
        }

        if (hoodAngle > maxHoodAngleDegrees) {
          maxHoodAngleDegrees = hoodAngle;
        }
      }

      this.minDistance = Inches.of(minDistanceInches);
      this.maxDistance = Inches.of(maxDistanceInches);
      this.minHoodAngle = Degrees.of(minHoodAngleDegrees);
      this.maxHoodAngle = Degrees.of(maxHoodAngleDegrees);

      this.hoodAngleFunction = interpolator.interpolate(distances, hoodAngles);
      this.flywheelVelocityFunction = interpolator.interpolate(distances, flywheelVelocities);
      this.distanceFunction = interpolator.interpolate(hoodAngles, distances);
      this.flightTimeFunction = interpolator.interpolate(distances, flightTimes);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Calculates the hood angle required to score given a distance to the target. Clamps
   * the output to the maximum and minimum hood angles if the input distance is outside the valid
   * range.
   *
   * @param distance the distance to the target
   * @return an Angle representing the hood angle needed for the shot
   */
  public Angle getHoodAngle(Distance distance) {
    return Degrees.of(
          hoodAngleFunction.value(
              MeasureUtil.clamp(distance, minDistance, maxDistance).in(Inches)));
  }

  /**
   * Predicts the distance to the target based on the given hood angle, assuming that the flywheel
   * is at the right speed. Clamps the input hood angle to the maximum and minimum hood angles if
   * it is outside the valid range.
   * 
   * @param hoodAngle the hood angle
   * @return a Distance representing the predicted distance to the target based on the hood angle
   */
  public Distance getDistance(Angle hoodAngle) {
    return Inches.of(distanceFunction.value(MeasureUtil.clamp(hoodAngle, minHoodAngle, maxHoodAngle).in(Degrees)));
  }

  /**
   * Calculates the required flywheel angular velocity to reach a target at the specified distance
   * and shooter angle. Clamps the output to the maximum and minimum flywheel velocities if the
   * input distance is outside the valid range.
   *
   * @param distance the distance to the target
   * @return the required flywheel angular velocity as an AngularVelocity (rotations per second)
   */
  public AngularVelocity getFlywheelVelocity(Distance distance) {
    if (isDistanceWithinValidRange(distance)) {
      return RotationsPerSecond.of(flywheelVelocityFunction.value(distance.in(Inches)));
    } else
      return RotationsPerSecond.of(
          flywheelVelocityFunction.value(
              MeasureUtil.clamp(distance, minDistance, maxDistance).in(Inches)));
  }

  /**
   * Calculates the flight time for a shot given the current distance to the target. Clamps the input
   * distance to the maximum and minimum valid distances if it is outside the valid range.
   * 
   * @param distance the distance to the target
   * @return the flight time for the shot as a Time
   */
  public Time getFlightTime(Distance distance) {
    return Seconds.of(
        flightTimeFunction.value(
            MeasureUtil.clamp(distance, minDistance, maxDistance).in(Inches)));
  }

  /**
   * Determines whether the given distance falls within the configured valid range.
   *
   * @param distance the Distance to check
   * @return true if the distance is greater than or equal to minDistance and less than
   *     or equal to maxDistance
   */
  public boolean isDistanceWithinValidRange(Distance distance) {
    return distance.gte(minDistance) && distance.lte(maxDistance);
  }
}
