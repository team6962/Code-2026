package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.team6962.lib.math.CSVLoader;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import java.io.IOException;
import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.interpolation.MicrosphereInterpolator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;

public class ShooterFunctions {

  private static final String anglePath = "hoodangledata.csv";
  private static final String velocityPath = "flywheelvelocitydata.csv";

  private MultivariateFunction hoodAngleFunction;
  private UnivariateFunction flywheelVelocityFunction;

  public ShooterFunctions() {
    try {
      this.hoodAngleFunction = loadShooterData();
      this.flywheelVelocityFunction = loadFlywheelData();
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
  private MultivariateFunction loadShooterData() throws IOException {
    MicrosphereInterpolator interpolator = new MicrosphereInterpolator();
    double[][] data = CSVLoader.loadCSV(anglePath);
    double[][] x = new double[2][data.length];
    for (int i = 0; i < data.length; i++) {
      x[0][i] = data[i][0];
      x[1][i] = data[i][1];
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
  private UnivariateFunction loadFlywheelData() throws IOException {
    SplineInterpolator interpolator = new SplineInterpolator();
    double[][] data = CSVLoader.loadCSV(velocityPath);
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
   * Returns the MultivariateFunction used by the shooter subsystem to compute shooter setpoints.
   *
   * @return the configured MultivariateFunction used to compute shooter setpoints, or {@code null}
   *     if no function has been configured
   */
  public MultivariateFunction getHoodAngleFunction() {
    return hoodAngleFunction;
  }

  /**
   * Calculates the hood angle required to score given the current distance to the target and the
   * shooter wheel angular velocity.
   *
   * @param distance the distance to the target (will be converted to inches for the internal
   *     calculation)
   * @param velocity the shooter wheel angular velocity (will be converted to rotations per second
   *     for the internal calculation)
   * @return an Angle representing the hood setpoint in degrees needed for the shot
   */
  public Angle getHoodAngle(Distance distance, AngularVelocity velocity) {
    return Degrees.of(
        hoodAngleFunction.value(
            new double[] {distance.in(Inches), velocity.in(RotationsPerSecond)}));
  }

  /**
   * Calculates the required flywheel angular velocity to reach a target at the specified distance
   * and shooter angle.
   *
   * @param distance the distance to the target; will be converted to inches before evaluation
   * @param angle the shooter/target angle (currently unused by this method)
   * @return the required flywheel angular velocity as an AngularVelocity (rotations per second)
   */
  public AngularVelocity getFlywheelVelocity(Distance distance, Angle angle) {
    return RotationsPerSecond.of(flywheelVelocityFunction.value(distance.in(Inches)));
  }
}
