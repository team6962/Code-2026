package com.team6962.lib.swerve.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.localization.Gyroscope;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Simulates a Pigeon2 gyroscope by computing heading changes from swerve
 * module motion.
 *
 * <p>Rather than simulating actual IMU physics, this class uses swerve
 * kinematics to calculate the robot's rotation from module position deltas.
 * This approach accurately reflects what the robot would experience during
 * normal driving, though it cannot simulate external forces like being
 * pushed. The accelerometer is not simulated and the gyroscope mounting is
 * assumed to be flat (no pitch or roll).
 *
 * <p>The {@link #update(double)} method calculates the twist (change in
 * pose) from module positions and applies the rotational component to the
 * gyroscope simulation. The yaw setter accounts for the gyroscope's
 * configured mount pose offset.
 *
 * <p>Additional setter methods allow manual override of pitch, roll, and
 * angular velocities for testing scenarios that require specific gyroscope
 * readings.
 */
public class GyroscopeSim {
    /** Drivetrain configuration containing gyroscope mount pose. */
    private DrivetrainConstants constants;
    /** Simulation state of the Pigeon2 gyroscope. */
    private Pigeon2SimState gyroSim;
    /** Odometry simulation used to derive heading changes. */
    private OdometrySim odometrySim;

    /**
     * Creates a new gyroscope simulation.
     *
     * @param constants drivetrain configuration
     * @param gyro the gyroscope to simulate
     * @param modulesSims module simulations to derive heading from
     */
    public GyroscopeSim(DrivetrainConstants constants, Gyroscope gyro, OdometrySim odometrySim) {
        this.constants = constants;
        this.gyroSim = gyro.getPigeon().getSimState();
        this.odometrySim = odometrySim;
    }

    /**
     * Updates the gyroscope simulation by calculating rotation from module
     * motion.
     *
     * @param deltaTimeSeconds time elapsed since the last update
     */
    public void update(double deltaTimeSeconds) {
        gyroSim.setRawYaw(odometrySim.getPosition().getRotation().getMeasure().plus(Degrees.of(constants.Gyroscope.DeviceConfiguration.MountPose.MountPoseYaw)));
        gyroSim.setAngularVelocityZ(RadiansPerSecond.of(odometrySim.getVelocity().omegaRadiansPerSecond));
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
