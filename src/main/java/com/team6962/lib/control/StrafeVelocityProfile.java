package com.team6962.lib.control;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Controls the strafe (perpendicular to the path) velocity of a drivetrain as it follows a path
 * between two points. The behavior is as followes:
 *
 * <ul>
 *   <li>Decelerate from the initial velocity to zero at the maximum acceleration, starting at the
 *       beginning of the path.
 *   <li>Use a scaled trapezoidal motion profile to slowly move to the mininum displacement from the
 *       path that will let the robot reach the target velocity at the end of the path.
 *   <li>Accelerate to the final velocity at the maximum acceleration, starting at the end of the
 *       path.
 * </ul>
 *
 * <p>There are two main cases of this behavior, which can luckily be handled with the same code:
 *
 * <ul>
 *   <li>If the initial velocity is in the opposite direction of the target velocity, the robot will
 *       move in a C shape around the path.
 *   <li>If the initial velocity is in the same direction as the target velocity, the robot will
 *       move in an S shape around the path.
 * </ul>
 */
public class StrafeVelocityProfile {
  private double maxAcceleration;
  private TrapezoidProfile profile;

  public StrafeVelocityProfile(TrapezoidProfile.Constraints constraints) {
    maxAcceleration = constraints.maxAcceleration;
    profile = new TrapezoidProfile(constraints);
  }

  public TrapezoidProfile.State calculate(
      double initialVelocity, double targetVelocity, double currentTime, double totalDuration) {
    double initialAccelerationTime = Math.abs(initialVelocity) / maxAcceleration;
    double finalAccelerationTime = Math.abs(targetVelocity) / maxAcceleration;

    if (currentTime < initialAccelerationTime) {
      // Decelerate from the initial velocity to zero
      return new TrapezoidProfile.State(
          initialVelocity * currentTime
              - 0.5 * Math.signum(initialVelocity) * maxAcceleration * currentTime * currentTime,
          -Math.signum(initialVelocity) * maxAcceleration * currentTime);
    } else if (currentTime > totalDuration - finalAccelerationTime) {
      double timeSinceStationary = currentTime - (totalDuration - finalAccelerationTime);
      // Accelerate from zero to the target velocity
      return new TrapezoidProfile.State(
          0.5
                  * Math.signum(targetVelocity)
                  * maxAcceleration
                  * timeSinceStationary
                  * timeSinceStationary
              - 0.5
                  * Math.signum(targetVelocity)
                  * maxAcceleration
                  * finalAccelerationTime
                  * finalAccelerationTime,
          Math.signum(targetVelocity) * maxAcceleration * timeSinceStationary);
    } else {
      double initialDisplacement =
          initialVelocity * initialAccelerationTime
              - 0.5
                  * Math.signum(initialVelocity)
                  * maxAcceleration
                  * initialAccelerationTime
                  * initialAccelerationTime;
      double finalDisplacement =
          -0.5
              * Math.signum(targetVelocity)
              * maxAcceleration
              * finalAccelerationTime
              * finalAccelerationTime;

      TrapezoidProfile.State initialState = new TrapezoidProfile.State(initialDisplacement, 0);
      TrapezoidProfile.State goalState = new TrapezoidProfile.State(finalDisplacement, 0);

      profile.calculate(currentTime, initialState, goalState);

      double profileTime = profile.totalTime();
      double intendedTime = totalDuration - initialAccelerationTime - finalAccelerationTime;

      TrapezoidProfile.State profileState =
          profile.calculate(currentTime / intendedTime * profileTime, initialState, goalState);

      return new TrapezoidProfile.State(
          profileState.position, profileState.velocity / intendedTime * profileTime);
    }
  }
}
