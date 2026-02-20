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

  private static final String path = "hoodangledata.csv";
  private static final String path2 = "flywheelvelocitydata.csv";

  private MultivariateFunction shooterFunction;
  private UnivariateFunction flywheelFunction;

  public ShooterFunctions() {
    try {
      this.shooterFunction = loadShooterData();
      this.flywheelFunction = loadFlywheelData();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private MultivariateFunction loadShooterData() throws IOException {
    MicrosphereInterpolator interpolator = new MicrosphereInterpolator();
    double[][] data = CSVLoader.loadCSV(path);
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

  private UnivariateFunction loadFlywheelData() throws IOException {
    SplineInterpolator interpolator = new SplineInterpolator();
    double[][] data = CSVLoader.loadCSV(path2);
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

  public MultivariateFunction getShooterFunction() {
    return shooterFunction;
  }

  public Angle getHoodAngle(Distance distance, AngularVelocity velocity) {
    return Degrees.of(
        shooterFunction.value(new double[] {distance.in(Inches), velocity.in(RotationsPerSecond)}));
  }

  public AngularVelocity getFlywheelVelocity(Distance distance, Angle angle) {
    return RotationsPerSecond.of(flywheelFunction.value(distance.in(Inches)));
  }
}
