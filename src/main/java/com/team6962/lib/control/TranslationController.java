package com.team6962.lib.control;

import static edu.wpi.first.units.Units.Meters;

import com.team6962.lib.math.MatrixUtil;
import com.team6962.lib.math.TranslationalVelocity;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Frequency;

/**
 * Uses trapezoidal motion profiling on two axis to control the position of a robot or mechanism in
 * 2D space. One profile controls the motion of the system along the direct line to the goal, while
 * the other profile controls motion perpendicular to that line (strafing).
 *
 * <h3>Control Method</h3>
 *
 * The controller provides the velocity that the system should be targeting at each time step in
 * order to follow the motion profile to the goal position. It uses a combination of the feedforward
 * velocity given by a motion profile and PID controllers to compute the output velocity along each
 * axis.
 *
 * <h3>Usage</h3>
 *
 * In the constructor, the PID constants, motion profile constraints, and update frequency must be
 * provided. Then, the {@link #setProfile setProfile()} method can be called to set up a motion
 * profile between two states (positions and velocities) that can later be followed. The {@link
 * #calculate calculate()} method can be called each time step to get the target velocity based on
 * the current position and velocity of the system. The {@link #isFinished()} method returns true
 * when the motion profile is completed, indicating that the system has reached the goal position
 * and velocity.
 *
 * <p>Additionally, the profile duration (the time it will probably take for the system to reach its
 * target) can be retrieved using the {@link #getDuration()} method. The {@link #setDuration
 * setDuration()} method can be used to scale the motion profiles to make the system take longer to
 * reach its goal.
 *
 * <h3>Internal Coordinate System</h3>
 *
 * The controller internally uses a coordinate system aligned with the path from the initial
 * position to the goal position. The "direct" axis is aligned with this path, while the "strafe"
 * axis is perpendicular to it (to the left). When working with vectors, Translation2d objects, and
 * TranslationalVelocity objects, the x component corresponds to the direct axis and the y component
 * corresponds to the strafe axis.
 */
public class TranslationController {
  /** The trapezoidal controller that moves the system along the direct line to the goal. */
  private TrapezoidalController directController;

  /**
   * The trapezoidal controller that moves the system perpendicular to the direct line to the goal
   * (strafing). This is used in many cases when the initial velocity or target velocity is nonzero.
   */
  private TrapezoidalController strafeController;

  /** The initial translation of the system in 2D space. */
  private Translation2d initialPosition;

  /** The goal translation of the system in 2D space. */
  private Translation2d goalPosition;

  /**
   * Constructs a TranslationController with the given PID constants, motion profile constraints,
   * and update frequency. The controller can be used to find velocity setpoints along a pair of
   * trapezoidal motion profiles that translate a robot or mechanism to a target in 2D space. The
   * same PID constants and motion profile constraints are used for both axes.
   *
   * @param kP The proportional feedback gain
   * @param kI The integral feedback gain
   * @param kD The derivative feedback gain
   * @param constraints The motion profile constraints (maximum velocity and acceleration)
   * @param updateFrequency The frequency at which the controller's {@link #calculate calculate()}
   *     method will be called
   */
  public TranslationController(
      double kP,
      double kI,
      double kD,
      TrapezoidProfile.Constraints constraints,
      Frequency updateFrequency) {
    // Initialize controllers that use trapezoidal profiling and PID to
    // reach targets along each axis
    this.directController = new TrapezoidalController(kP, kI, kD, constraints, updateFrequency);
    this.strafeController = new TrapezoidalController(kP, kI, kD, constraints, updateFrequency);
  }

  /**
   * Sets up the motion profiles to move the system from an initial position and velocity to a goal
   * position and velocity in 2D space. Two trapezoidal motion profiles are created separately for
   * each axis, with one profile controlling motion along the direct line to the goal, and the other
   * profile controlling motion perpendicular to that line (strafing). After setting the profiles,
   * the durations of the two profiles are synchronized so that they complete at the same time.
   *
   * @param initialPosition The initial translation of the system
   * @param initialVelocity The initial translational velocity of the system
   * @param goalPosition The goal translation of the system
   * @param goalVelocity The goal translational velocity of the system
   */
  public void setProfile(
      Translation2d initialPosition,
      TranslationalVelocity initialVelocity,
      Translation2d goalPosition,
      TranslationalVelocity goalVelocity) {
    this.initialPosition = initialPosition;
    this.goalPosition = goalPosition;

    // The following code transforms the initial and goal velocities into a
    // coordinate systems used by the motion profiles
    Vector<N2> fieldRelativeInitialVelocityVector = initialVelocity.toVector();
    Vector<N2> fieldRelativeGoalVelocityVector = goalVelocity.toVector();

    // Create a rotation matrix to convert from field-relative to
    // path-relative velocities
    Matrix<N2, N2> fieldToPathVelocityMatrix =
        MatrixUtil.toMatrix2x2(getFieldToPathMatrix(initialPosition, goalPosition));

    // Transform the initial and goal velocities into path-relative velocities
    Vector<N2> pathRelativeInitialVelocityVector =
        new Vector<>(fieldToPathVelocityMatrix.times(fieldRelativeInitialVelocityVector));
    Vector<N2> pathRelativeGoalVelocityVector =
        new Vector<>(fieldToPathVelocityMatrix.times(fieldRelativeGoalVelocityVector));

    // Set the initial and final states for each controller
    directController.setProfile(
        new TrapezoidProfile.State(
            -goalPosition.minus(initialPosition).getNorm(),
            pathRelativeInitialVelocityVector.get(0)),
        new TrapezoidProfile.State(0, pathRelativeGoalVelocityVector.get(0)));

    strafeController.setProfile(
        new TrapezoidProfile.State(0, pathRelativeInitialVelocityVector.get(1)),
        new TrapezoidProfile.State(0, pathRelativeGoalVelocityVector.get(1)));

    // Synchronize durations of the two motion profiles
    setDuration(getDuration());
  }

  /**
   * Gets the duration of the motion profile being followed, in seconds.
   *
   * @return The duration of the motion profile being followed
   */
  public double getDuration() {
    // The overall duration is the longer of the two profile durations.
    // In most cases, these durations will be equal due to synchronization,
    // but they could be different when one axis has no movement.
    return Math.max(directController.getDuration(), strafeController.getDuration());
  }

  /**
   * Scales the durations of the motion profiles being followed so that they both take the given
   * amount of time to complete. This method effectively adjusts the maximum velocities and
   * accelerations of the profiles to achieve the new duration. Note that durations less than the
   * value returned by {@link #getDuration()} may result in profiles that are not feasible within
   * the original constraints.
   *
   * @param duration The desired duration of the motion profiles
   */
  public void setDuration(double duration) {
    directController.setDuration(duration);
    strafeController.setDuration(duration);
  }

  /**
   * Calculates the target translational velocity for the system at the current time step, based on
   * the current position and velocity of the system.
   *
   * @param currentPosition The current translation of the system
   * @param currentVelocity The current translational velocity of the system
   * @return The target translational velocity for the system
   */
  public TranslationalVelocity calculate(
      Translation2d currentPosition, TranslationalVelocity currentVelocity) {
    // Convert the current position to path-relative coordinates
    Matrix<N3, N3> fieldToPathPositionMatrix = getFieldToPathMatrix(initialPosition, goalPosition);

    Vector<N3> pathRelativeCurrentPosition =
        new Vector<>(
            fieldToPathPositionMatrix.times(MatrixUtil.toVector3(currentPosition.toVector())));

    // Convert the current velocity to path-relative coordinates
    Matrix<N2, N2> fieldToPathVelocityMatrix =
        MatrixUtil.toMatrix2x2(getFieldToPathMatrix(initialPosition, goalPosition));

    Vector<N2> pathRelativeCurrentVelocity =
        new Vector<>(fieldToPathVelocityMatrix.times(currentVelocity.toVector()));

    // Calculate the target velocity in path-relative coordinates
    Vector<N2> pathRelativeTargetVelocity =
        calculatePathCoordinates(pathRelativeCurrentPosition, pathRelativeCurrentVelocity);

    // Convert the target velocity back to field-relative coordinates
    Matrix<N2, N2> pathToFieldVelocityMatrix =
        MatrixUtil.toMatrix2x2(getPathToFieldMatrix(initialPosition, goalPosition));

    TranslationalVelocity fieldRelativeTargetVelocity =
        new TranslationalVelocity(pathToFieldVelocityMatrix.times(pathRelativeTargetVelocity));

    return fieldRelativeTargetVelocity;
  }

  /**
   * Calculates the target velocity in path-relative coordinates based on the current position and
   * velocity in path-relative coordinates.
   *
   * @param currentPosition The current position in path-relative coordinates
   * @param currentVelocity The current velocity in path-relative coordinates
   * @return The target velocity in path-relative coordinates
   */
  private Vector<N2> calculatePathCoordinates(
      Vector<?> currentPosition, Vector<?> currentVelocity) {
    // Use the trapezoidal controllers to calculate target velocities
    // along each axis
    double directTargetVelocity =
        directController.calculate(
            new TrapezoidProfile.State(
                Meters.of(currentPosition.get(0)).in(Meters), currentVelocity.get(0)));

    double strafeTargetVelocity =
        strafeController.calculate(
            new TrapezoidProfile.State(
                Meters.of(currentPosition.get(1)).in(Meters), currentVelocity.get(1)));

    return new TranslationalVelocity(directTargetVelocity, strafeTargetVelocity).toVector();
  }

  /**
   * Returns true if the motion profiles have finished executing, meaning the system has likely
   * reached its goal position and velocity. If there were significant physical disturbances, the
   * system may not be exactly at the goal when this method returns true.
   *
   * @return True if the motion profiles are finished
   */
  public boolean isFinished() {
    return directController.isFinished() && strafeController.isFinished();
  }

  /**
   * Gets the 3x3 transformation matrix that converts coordinates from the path-relative frame to
   * the field-relative frame. This includes a rotation and translation to align with the path from
   * the initial position to the goal position. The 2x2 upper left corner of this matrix can be used
   * on its own to transform 2-component velocity vectors.
   *
   * <p>This is the inverse of the matrix returned by {@link #getFieldToPathMatrix
   * getFieldToPathMatrix()}.
   *
   * @param initialPosition The initial position
   * @param goalPosition The goal position
   * @return The path-to-field 3x3 transformation matrix
   */
  private static Matrix<N3, N3> getPathToFieldMatrix(
      Translation2d initialPosition, Translation2d goalPosition) {
    Vector<N2> pathDirectionVector = getPathDirectionVector(initialPosition, goalPosition);
    Matrix<N3, N3> pathToFieldRotationMatrix = MatrixUtil.createRotation3x3(pathDirectionVector);
    Matrix<N3, N3> translationMatrix = MatrixUtil.createTranslation3x3(goalPosition.toVector());

    return translationMatrix.times(pathToFieldRotationMatrix);
  }

  /**
   * Gets the 3x3 transformation matrix that converts coordinates from the field-relative frame to
   * the path-relative frame. This includes a rotation and translation to align with the path from
   * the initial position to the goal position. The 2x2 upper left corner of this matrix can be used
   * on its own to transform 2-component velocity vectors.
   *
   * <p>This is the inverse of the matrix returned by {@link #getPathToFieldMatrix
   * getPathToFieldMatrix()}.
   *
   * @param initialPosition The initial position
   * @param goalPosition The goal position
   * @return The field-to-path 3x3 transformation matrix
   */
  private static Matrix<N3, N3> getFieldToPathMatrix(
      Translation2d initialPosition, Translation2d goalPosition) {
    return getPathToFieldMatrix(initialPosition, goalPosition).inv();
  }

  /**
   * Gets the unit vector pointing in the direction from the initial position to the goal position.
   *
   * @param initialPosition The initial position
   * @param goalPosition The goal position
   * @return The unit vector pointing from the initial position to the goal position
   */
  private static Vector<N2> getPathDirectionVector(
      Translation2d initialPosition, Translation2d goalPosition) {
    return goalPosition.minus(initialPosition).toVector().unit();
  }
}
