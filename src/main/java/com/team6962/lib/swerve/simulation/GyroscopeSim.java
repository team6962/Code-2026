package com.team6962.lib.swerve.simulation;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Gyroscope;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/**
 * Simulates a Pigeon2 gyroscope by computing robot yaw from MapleSim's physics simulation of the
 * robot's motion.
 *
 * <p>Additional setter methods allow manual override of pitch, roll, and angular velocities for
 * testing scenarios that require specific gyroscope readings.
 */
public class GyroscopeSim {
  /** Drivetrain configuration containing gyroscope mount pose. */
  private DrivetrainConstants constants;

  /** Simulation state of the Pigeon2 gyroscope. */
  private Pigeon2SimState gyroSim;

  /** MapleSim instance to get robot yaw and angular velocity for simulating gyro readings. */
  private GyroSimulation gyroSimulation;

  /**
   * Creates a new gyroscope simulation.
   *
   * @param constants drivetrain configuration
   * @param gyro the gyroscope to simulate
   * @param modulesSims module simulations to derive heading from
   * @param mapleSim the MapleSim instance to get robot yaw and angular velocity for simulating gyro
   *     readings
   */
  public GyroscopeSim(DrivetrainConstants constants, Gyroscope gyro, MapleSim mapleSim) {
    this.constants = constants;
    this.gyroSim = gyro.getPigeon().getSimState();
    this.gyroSimulation = mapleSim.getSwerveSim().getGyroSimulation();
  }

  /**
   * Updates the gyroscope simulation from the MapleSim physics simulation. This should be called
   * periodically (typically every simulation tick) after updating the MapleSim arena.
   *
   * @param deltaTimeSeconds time elapsed since the last update
   */
  public void update(double deltaTimeSeconds) {
    gyroSim.setRawYaw(
        gyroSimulation
            .getGyroReading()
            .getMeasure()
            .plus(Degrees.of(constants.Gyroscope.DeviceConfiguration.MountPose.MountPoseYaw)));
    gyroSim.setAngularVelocityZ(gyroSimulation.getMeasuredAngularVelocity());
  }

  /**
   * Sets the gyroscope pitch.
   *
   * @param pitch the pitch angle to set
   */
  public void setPitch(Angle pitch) {
    gyroSim.setPitch(pitch);
  }

  /**
   * Sets the gyroscope roll.
   *
   * @param roll the roll angle to set
   */
  public void setRoll(Angle roll) {
    gyroSim.setRoll(roll);
  }

  /**
   * Sets the gyroscope pitch angular velocity.
   *
   * @param pitchVelocity the pitch velocity to set
   */
  public void setPitchVelocity(AngularVelocity pitchVelocity) {
    gyroSim.setAngularVelocityY(pitchVelocity);
  }

  /**
   * Sets the gyroscope roll angular velocity.
   *
   * @param rollVelocity the roll velocity to set
   */
  public void setRollVelocity(AngularVelocity rollVelocity) {
    gyroSim.setAngularVelocityX(rollVelocity);
  }
}
