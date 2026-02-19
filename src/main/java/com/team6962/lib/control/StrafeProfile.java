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
public class StrafeProfile implements MotionProfile {
  private double maxAcceleration;
  private TrapezoidalProfile trapezoidalProfile;
  private double initialVelocity, targetVelocity;

  public StrafeProfile(TrapezoidProfile.Constraints constraints) {
    maxAcceleration = constraints.maxAcceleration;
    trapezoidalProfile = new TrapezoidalProfile(constraints);
  }

  private double getInitialAccelerationTime() {
    return Math.abs(initialVelocity) / maxAcceleration;
  }

  private double getFinalAccelerationTime() {
    return Math.abs(targetVelocity) / maxAcceleration;
  }

  private double getInitialDisplacement() {
    double initialAccelerationTime = getInitialAccelerationTime();
    return initialVelocity * initialAccelerationTime
        - 0.5
            * Math.signum(initialVelocity)
            * maxAcceleration
            * initialAccelerationTime
            * initialAccelerationTime;
  }

  private double getFinalDisplacement() {
    double finalAccelerationTime = getFinalAccelerationTime();
    return -0.5
        * Math.signum(targetVelocity)
        * maxAcceleration
        * finalAccelerationTime
        * finalAccelerationTime;
  }

  @Override
  public void setDuration(double duration) {
    trapezoidalProfile.setDuration(
        duration - getInitialAccelerationTime() - getFinalAccelerationTime());
  }

  @Override
  public double getDuration() {
    return trapezoidalProfile.getDuration()
        + getInitialAccelerationTime()
        + getFinalAccelerationTime();
  }

  @Override
  public void setProfile(State initial, State goal) {
    if (Math.abs(initial.position) > 1e-6 || Math.abs(goal.position) > 1e-6) {
      throw new IllegalArgumentException(
          "StrafeVelocityProfile only supports profiles between states with zero displacement");
    }

    initialVelocity = initial.velocity;
    targetVelocity = goal.velocity;

    trapezoidalProfile.setProfile(
        new MotionProfile.State(getInitialDisplacement(), 0),
        new MotionProfile.State(getFinalDisplacement(), 0));
  }

  @Override
  public MotionProfile.State sampleAt(double currentTime) {
    double initialAccelerationTime = Math.abs(initialVelocity) / maxAcceleration;
    double finalAccelerationTime = Math.abs(targetVelocity) / maxAcceleration;
    double totalDuration = getDuration();

    if (currentTime < initialAccelerationTime) {
      // Decelerate from the initial velocity to zero
      return new MotionProfile.State(
          initialVelocity * currentTime
              - 0.5 * Math.signum(initialVelocity) * maxAcceleration * currentTime * currentTime,
          initialVelocity - Math.signum(initialVelocity) * maxAcceleration * currentTime);
    } else if (currentTime > totalDuration - finalAccelerationTime) {
      double timeSinceStationary = currentTime - (totalDuration - finalAccelerationTime);
      // Accelerate from zero to the target velocity
      return new MotionProfile.State(
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
      return trapezoidalProfile.sampleAt(currentTime - initialAccelerationTime);
    }
  }
}
