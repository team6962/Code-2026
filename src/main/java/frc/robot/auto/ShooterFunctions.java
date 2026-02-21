package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.team6962.lib.math.CSVLoader;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import java.io.IOException;
import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.interpolation.MicrosphereInterpolator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;

public class ShooterFunctions {
  /**
   * Path to the CSV file containing 2D hood angle calibration data for hub shots. Formatted as
   * Distance, Velocity, Angle
   */
  private static final String anglePathHub = "hoodangledatahub.csv";

  /**
   * Path to the CSV file containing 2-column flywheel velocity calibration data for hub shots.
   * Formatted as Distance, Velocity
   */
  private static final String velocityPathHub = "flywheelvelocitydatahub.csv";

  /**
   * Path to the CSV file containing hood angle calibration data for passing. Formatted as Distance,
   * Angle
   */
  private static final String anglePathPass = "passinghoodangledata.csv";

  /**
   * Path to the CSV file containing flywheel velocity calibration data for passing. Formatted as
   * Distance, Velocity
   */
  private static final String velocityPathPass = "passingflywheelvelocitydata.csv";

  /**
   * Path to the CSV file containing minimum and maximum velocities to shoot at. Formatted as
   * Distance, Velocity, Angle
   */
  private static final String velocityRangeHub = "velocityrangehub.csv";

  /** Minimum valid distance to the target for hub shots, in inches. */
  private static final double minDistanceHub = 57.5; // Calibrated for our shooter A testing

  /** Maximum valid distance to the target for hub shots, in inches. */
  private static final double maxDistanceHub = 263.0; // Calibrated for our shooter A testing

  /** Minimum valid distance to the target for passing, in inches. */
  private static final double minDistancePass = 57.5; // Needs to be changed

  /** Maximum valid distance to the target for passing, in inches. */
  private static final double maxDistancePass = 263.0; // Needs to be changed

  private MultivariateFunction hoodAngleFunctionHub;
  private UnivariateFunction flywheelVelocityFunctionHub;
  private UnivariateFunction hoodAngleFunctionPass;
  private UnivariateFunction flywheelVelocityFunctionPass;
  private UnivariateFunction minVelocityHub;
  private UnivariateFunction maxVelocityHub;

  public ShooterFunctions() {
    try {
      this.hoodAngleFunctionHub = loadHoodAngleDataHub();
      this.flywheelVelocityFunctionHub = loadFlywheelDataHub();
      this.hoodAngleFunctionPass = loadHoodAngleDataPass();
      this.flywheelVelocityFunctionPass = loadFlywheelDataPass();
      this.minVelocityHub = loadMinVelocityHub();
      this.maxVelocityHub = loadMaxVelocityHub();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Loads shooter calibration data from a CSV file and returns an interpolating function.
   *
   * <p>The returned {@code MultivariateFunction} accepts a {@code double[]} of length 2 and returns
   * an interpolated scalar value for that 2D input.
   *
   * @return a {@code MultivariateFunction} that interpolates the 2-dimensional input to a scalar
   *     output using the data loaded from the CSV file
   * @throws IOException if an I/O error occurs while reading the CSV file at {@code path}
   */
  private MultivariateFunction loadHoodAngleDataHub() throws IOException {
    MicrosphereInterpolator interpolator = new MicrosphereInterpolator();
    double[][] data = CSVLoader.loadCSV(anglePathHub);
    double[][] x = new double[data.length][2];
    for (int i = 0; i < data.length; i++) {
      x[i][0] = data[i][0];
      x[i][1] = data[i][1];
    }
    double[] y = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      y[i] = data[i][2];
    }
    return interpolator.interpolate(x, y);
  }

  /**
   * Loads two-column numeric data from the CSV file referenced by the field {@code path2} and
   * returns a spline-based interpolating function.
   *
   * @return an {@code UnivariateFunction} representing the spline interpolation of the CSV data
   *     (f(x) ≈ y)
   * @throws IOException if an I/O error occurs while reading the CSV file at {@code path2}
   */
  private UnivariateFunction loadFlywheelDataHub() throws IOException {
    SplineInterpolator interpolator = new SplineInterpolator();
    double[][] data = CSVLoader.loadCSV(velocityPathHub);
    double[] x = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      x[i] = data[i][0];
    }
    double[] y = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      y[i] = data[i][1];
    }
    return interpolator.interpolate(x, y);
  }

  /**
   * Loads shooter calibration data for passing from a CSV file and returns an interpolating
   * function.
   *
   * @return a {@code UnivariateFunction} that interpolates the distance to a hood angle setpoint
   *     using the data loaded from the CSV file
   * @throws IOException if an I/O error occurs while reading the CSV file at {@code anglePathPass}
   */
  private UnivariateFunction loadHoodAngleDataPass() throws IOException {
    SplineInterpolator interpolator = new SplineInterpolator();
    double[][] data = CSVLoader.loadCSV(anglePathPass);
    double[] x = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      x[i] = data[i][0];
    }
    double[] y = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      y[i] = data[i][2];
    }
    return interpolator.interpolate(x, y);
  }

  /**
   * Loads flywheel velocity calibration data for passing from a CSV file and returns an
   * interpolating function.
   *
   * @return a {@code UnivariateFunction} that interpolates the distance to a flywheel velocity
   *     setpoint using the data loaded from the CSV file
   * @throws IOException if an I/O error occurs while reading the CSV file at {@code
   *     velocityPathPass}
   */
  private UnivariateFunction loadFlywheelDataPass() throws IOException {
    SplineInterpolator interpolator = new SplineInterpolator();
    double[][] data = CSVLoader.loadCSV(velocityPathPass);
    double[] x = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      x[i] = data[i][0];
    }
    double[] y = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      y[i] = data[i][1];
    }
    return interpolator.interpolate(x, y);
  }

  /**
   * Loads minimum flywheel velocity data for hub shots from a CSV file and returns an interpolating
   * function.
   *
   * @return a {@code UnivariateFunction} that interpolates the distance to a velocity setpoint
   *     using the data loaded from the CSV file
   * @throws IOException if an I/O error occurs while reading the CSV file at {@code
   *     velocityRangeHub}
   */
  private UnivariateFunction loadMinVelocityHub() throws IOException {
    SplineInterpolator interpolator = new SplineInterpolator();
    double[][] data = CSVLoader.loadCSV(velocityRangeHub);
    double[] x = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      x[i] = data[i][0];
    }
    double[] y = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      y[i] = data[i][1];
    }
    return interpolator.interpolate(x, y);
  }

  /**
   * Loads maximum flywheel velocity data for hub shots from a CSV file and returns an interpolating
   * function.
   *
   * @return a {@code UnivariateFunction} that interpolates the distance to a velocity setpoint
   *     using the data loaded from the CSV file
   * @throws IOException if an I/O error occurs while reading the CSV file at {@code
   *     velocityRangeHub}
   */
  private UnivariateFunction loadMaxVelocityHub() throws IOException {
    SplineInterpolator interpolator = new SplineInterpolator();
    double[][] data = CSVLoader.loadCSV(velocityRangeHub);
    double[] x = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      x[i] = data[i][0];
    }
    double[] y = new double[data.length];
    for (int i = 0; i < data.length; i++) {
      y[i] = data[i][2];
    }
    return interpolator.interpolate(x, y);
  }

  /**
   * Returns the MultivariateFunction used by the shooter subsystem to compute hood angle setpoints.
   *
   * @return the configured MultivariateFunction used to compute hood angle setpoints, or {@code
   *     null} if no function has been configured
   */
  public MultivariateFunction getHoodAngleFunctionHub() {
    return hoodAngleFunctionHub;
  }

  /**
   * Returns the UnivariateFunction used by the shooter subsystem to compute flywheel velocity
   * setpoints.
   *
   * @return the configured UnivariateFunction used to compute flywheel velocity setpoints, or
   *     {@code null} if no function has been configured
   */
  public UnivariateFunction getFlywheelVelocityFunctionHub() {
    return flywheelVelocityFunctionHub;
  }

  /**
   * Returns the UnivariateFunction used by the shooter subsystem to compute hood angle setpoints
   * for passing.
   *
   * @return the configured UnivariateFunction used to compute hood angle setpoints for passing, or
   *     {@code null} if no function has been configured
   */
  public UnivariateFunction getHoodAngleFunctionPass() {
    return hoodAngleFunctionPass;
  }

  /**
   * Returns the UnivariateFunction used by the shooter subsystem to compute flywheel velocity
   * setpoints for passing.
   *
   * @return the configured UnivariateFunction used to compute flywheel velocity setpoints for
   *     passing, or {@code null} if no function has been configured
   */
  public UnivariateFunction getFlywheelVelocityFunctionPass() {
    return flywheelVelocityFunctionPass;
  }

  /**
   * Calculates the hood angle required to score given the current distance to the target and the
   * shooter wheel angular velocity. Clamps the output to the maximum and minimum hood angles if the
   * input distance is outside the valid range.
   *
   * @param distance the distance to the target (will be converted to inches for the internal
   *     calculation)
   * @param velocity the shooter wheel angular velocity (will be converted to rotations per second
   *     for the internal calculation)
   * @return an Angle representing the hood setpoint in degrees needed for the shot
   */
  public Angle getHoodAngleHub(Distance distance, AngularVelocity velocity) {
    if (isDistanceWithinValidHubRange(distance)
        && isVelocityWithinValidHubRange(distance, velocity)) {
      return Degrees.of(
          hoodAngleFunctionHub.value(
              new double[] {distance.in(Inches), velocity.in(RotationsPerSecond)}));
    } else if (isVelocityWithinValidHubRange(distance, velocity)) {
      return Degrees.of(
          hoodAngleFunctionHub.value(
              new double[] {
                MathUtil.clamp(distance.in(Inches), minDistanceHub, maxDistanceHub),
                velocity.in(RotationsPerSecond)
              }));
    } else
      return Degrees.of(
          hoodAngleFunctionHub.value(
              new double[] {
                MathUtil.clamp(distance.in(Inches), minDistanceHub, maxDistanceHub),
                MathUtil.clamp(
                    velocity.in(RotationsPerSecond),
                    minVelocityHub.value(distance.in(Inches)),
                    maxVelocityHub.value(distance.in(Inches)))
              }));
  }

  /**
   * Calculates the required flywheel angular velocity to reach a target at the specified distance
   * and shooter angle. Clamps the output to the maximum and minimum flywheel velocities if the
   * input distance is outside the valid range.
   *
   * @param distance the distance to the target; will be converted to inches before evaluation
   * @param angle the shooter/target angle (currently unused by this method)
   * @return the required flywheel angular velocity as an AngularVelocity (rotations per second)
   */
  public AngularVelocity getFlywheelVelocityHub(Distance distance) {
    if (isDistanceWithinValidHubRange(distance)) {
      return RotationsPerSecond.of(flywheelVelocityFunctionHub.value(distance.in(Inches)));
    } else
      return RotationsPerSecond.of(
          flywheelVelocityFunctionHub.value(
              MathUtil.clamp(distance.in(Inches), minDistanceHub, maxDistanceHub)));
  }

  /**
   * Calculates the hood angle required for passing given the current distance to the target. Clamps
   * the output to the maximum and minimum hood angles if the input distance is outside the valid
   * range.
   *
   * @param distance the distance to the target (will be converted to inches for the internal
   *     calculation)
   * @return an Angle representing the hood setpoint in degrees needed for passing
   */
  public Angle getHoodAnglePass(Distance distance) {
    if (isDistanceWithinValidPassRange(distance)) {
      return Degrees.of(hoodAngleFunctionPass.value(distance.in(Inches)));
    } else
      return Degrees.of(
          hoodAngleFunctionPass.value(
              MathUtil.clamp(distance.in(Inches), minDistancePass, maxDistancePass)));
  }

  /**
   * Calculates the required flywheel angular velocity for passing given the current distance to the
   * target. Clamps the output to the maximum and minimum flywheel velocities if the input distance
   * is outside the valid range.
   *
   * @param distance the distance to the target; will be converted to inches before evaluation
   * @return the required flywheel angular velocity for passing as an AngularVelocity (rotations per
   *     second)
   */
  public AngularVelocity getFlywheelVelocityPass(Distance distance) {
    if (isDistanceWithinValidPassRange(distance)) {
      return RotationsPerSecond.of(flywheelVelocityFunctionPass.value(distance.in(Inches)));
    } else
      return RotationsPerSecond.of(
          flywheelVelocityFunctionPass.value(
              MathUtil.clamp(distance.in(Inches), minDistancePass, maxDistancePass)));
  }

  /**
   * Determines whether the given distance falls within the configured valid range.
   *
   * @param distance the Distance to check; will be converted to inches for comparison
   * @return true if the distance (in inches) is greater than or equal to minDistance and less than
   *     or equal to maxDistance
   */
  public boolean isDistanceWithinValidHubRange(Distance distance) {
    double distanceInInches = distance.in(Inches);
    return distanceInInches >= minDistanceHub && distanceInInches <= maxDistanceHub;
  }
  /**
   * Determines whether the given velocity falls within the configured valid range.
   *
   * @param distance the Distance to check; will be converted to inches for comparison
   * @param velocity the AngularVelocity to check
   * @return true if the velocity is within the valid range for the given distance
   */
  public boolean isVelocityWithinValidHubRange(Distance distance, AngularVelocity velocity) {
    double velocityInRPS = velocity.in(RotationsPerSecond);
    return velocityInRPS >= minVelocityHub.value(distance.in(Inches))
        && velocityInRPS <= maxVelocityHub.value(distance.in(Inches));
  }

  /**
   * Determines whether the given distance falls within the configured valid range for passing.
   *
   * @param distance the Distance to check; will be converted to inches for comparison
   * @return true if the distance (in inches) is greater than or equal to minDistancePass and less
   *     than or equal to maxDistancePass
   */
  public boolean isDistanceWithinValidPassRange(Distance distance) {
    double distanceInInches = distance.in(Inches);
    return distanceInInches >= minDistancePass && distanceInInches <= maxDistancePass;
  }
}
