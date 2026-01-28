package frc.robot.controls.shoot;

import static edu.wpi.first.units.Units.Inches;

import com.team6962.lib.math.ConstantFunction;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.analysis.UnivariateFunction;

public class AutoShootConstants {
  /**
   * Function that takes in (distance to target, roller speed, target height) and returns hood
   * angle. Uses SI units.
   */
  public static final MultivariateFunction hoodAngleFunction = new ConstantFunction(0.125);

  /** Function that takes in (hood angle, roller speed) and returns release time. Uses SI units. */
  public static final MultivariateFunction releaseTimeFunction = new ConstantFunction(0.2);

  /**
   * Function that takes in (hood angle, roller speed, target height) and returns flight time. Uses
   * SI units.
   */
  public static final MultivariateFunction flightTimeFunction = new ConstantFunction(0.5);

  /**
   * Function that takes in (hood angle, roller speed, target height) and returns distance to
   * target. Uses SI units.
   */
  public static final MultivariateFunction distanceFunction = new ConstantFunction(4);

  /** Function that takes in (distance to target) and returns roller speed. Uses SI units. */
  public static final UnivariateFunction rollerSpeedFunction = new ConstantFunction(500);

  /** Maximum allowable error of the ideal shooting solution found by optimization. */
  public static final Distance optimizationTolerance = Inches.of(0.1);

  /** Maximum allowable error to shoot. */
  public static final Distance shootingTolerance = Inches.of(6);

  /** Transform representing the shooter's position and orientation relative to the robot. */
  public static final Transform2d shooterTransform = new Transform2d();
}
