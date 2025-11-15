package com.team6962.lib.swerve.localization;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team6962.lib.phoenix.StatusUtil;
import com.team6962.lib.swerve.config.DrivetrainConstants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Provides an interface to the Pigeon 2.0 gyroscope with methods to get
 * latency-compensated orientations and angular velocities.
 */
public class Gyroscope {
    /**
     * The drivetrain constants containing gyroscope configuration.
     */
    private DrivetrainConstants constants;

    /**
     * The Pigeon 2.0 gyroscope instance. This communicates with the physical
     * gyro hardware over CAN bus.
     */
    private Pigeon2 gyro;

    // Status signals for yaw, pitch, roll, and angular velocities around each
    // of those axes
    private StatusSignal<Angle> yawSignal;
    private StatusSignal<AngularVelocity> yawVelocitySignal;

    private StatusSignal<Angle> pitchSignal;
    private StatusSignal<AngularVelocity> pitchVelocitySignal;

    private StatusSignal<Angle> rollSignal;
    private StatusSignal<AngularVelocity> rollVelocitySignal;

    /**
     * Constructs a Gyroscope object using the provided drivetrain constants.
     * 
     * @param constants The drivetrain constants containing gyroscope
     * configuration
     */
    public Gyroscope(DrivetrainConstants constants) {
        this.constants = constants;

        gyro = new Pigeon2(constants.Gyroscope.CANId, constants.CANBusName);

        StatusUtil.check(gyro.getConfigurator().apply(constants.Gyroscope.DeviceConfiguration));

        yawSignal = gyro.getYaw();
        yawVelocitySignal = gyro.getAngularVelocityZWorld();

        pitchSignal = gyro.getPitch();
        pitchVelocitySignal = gyro.getAngularVelocityYWorld();

        rollSignal = gyro.getRoll();
        rollVelocitySignal = gyro.getAngularVelocityXWorld();

        BaseStatusSignal.setUpdateFrequencyForAll(constants.Timing.SignalUpdateRate, getStatusSignals());
    }

    /**
     * Gets all status signals used by the gyroscope.
     */
    public BaseStatusSignal[] getStatusSignals() {
        return new BaseStatusSignal[] {
            yawSignal,
            yawVelocitySignal,
            pitchSignal,
            pitchVelocitySignal,
            rollSignal,
            rollVelocitySignal
        };
    }

    /**
     * Gets the underlying Pigeon2 gyroscope instance.
     * 
     * @return The Pigeon2 instance
     */
    public Pigeon2 getPigeon() {
        return gyro;
    }

    /**
     * Gets the yaw angle of the robot.
     * 
     * @return The yaw angle
     */
    public Angle getYaw() {
        if (constants.Gyroscope.LatencyCompensation) {
            return Rotations.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                yawSignal,
                yawVelocitySignal
            ));
        } else {
            return yawSignal.getValue();
        }
    }

    /**
     * Gets the pitch angle of the robot.
     * 
     * @return The pitch angle
     */
    public Angle getPitch() {
        if (constants.Gyroscope.LatencyCompensation) {
            return Rotations.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                pitchSignal,
                pitchVelocitySignal
            ));
        } else {
            return pitchSignal.getValue();
        }
    }

    /**
     * Gets the roll angle of the robot.
     * 
     * @return The roll angle
     */
    public Angle getRoll() {
        if (constants.Gyroscope.LatencyCompensation) {
            return Rotations.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                rollSignal,
                rollVelocitySignal
            ));
        } else {
            return rollSignal.getValue();
        }
    }

    /**
     * Gets the yaw angular velocity of the robot.
     * 
     * @return The yaw angular velocity
     */
    public AngularVelocity getYawVelocity() {
        return yawVelocitySignal.getValue();
    }

    public AngularVelocity getPitchVelocity() {
        return pitchVelocitySignal.getValue();
    }

    public AngularVelocity getRollVelocity() {
        return rollVelocitySignal.getValue();
    }

    public void update() {
    }
}
