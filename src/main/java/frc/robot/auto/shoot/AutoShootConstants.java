package frc.robot.auto.shoot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.apache.commons.math3.analysis.MultivariateFunction;

import com.team6962.lib.math.ConstantFunction;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Constants for the AutoShoot command. These constants include functions that take in various
 * parameters and return the corresponding shooting parameters (hood angle, roller speed, etc).
 * These functions are used to calculate the optimal shooting parameters based on the current state
 * of the robot and the target.
 */
public class AutoShootConstants {
  /**
   * Function that takes in (target distance, hood angle) and returns initial velocity
   * displacement scalar. This function is used to account for drag and the Magnus effect when
   * shooting on the move. Uses inches and degrees as inputs.
   */
  public static final MultivariateFunction initialVelocityDisplacementScalarFunction = new ConstantFunction(0.9);

  /**
   * The number of iterations to run the optimization for when calculating the optimal shooting
   * parameters.
   */
  public static final int optimizationIterations = 20;

  /** Maximum allowable flywheel velocity error to shoot. */
  public static final AngularVelocity flywheelVelocityTolerance = RotationsPerSecond.of(0.25);

  /** Maximum allowable hood angle error to shoot. */
  public static final Angle hoodAngleTolerance = Degrees.of(1);

  /** Maximum allowable turret angle error to shoot. */
  public static final Angle turretAngleTolerance = Degrees.of(1);

  /** Transform representing the shooter's position and orientation relative to the robot. */
  public static final Transform2d shooterTransform = new Transform2d();
}
