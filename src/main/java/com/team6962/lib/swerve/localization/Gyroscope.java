package com.team6962.lib.swerve.localization;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.phoenix.StatusUtil;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.util.SwerveComponent;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Provides an interface to the Pigeon 2.0 gyroscope with methods to get
 * latency-compensated orientations and angular velocities.
 */
public class Gyroscope implements SwerveComponent, AutoCloseable {
    /**
     * The drivetrain constants containing gyroscope configuration.
     */
    private final DrivetrainConstants constants;

    /**
     * The Pigeon 2.0 gyroscope instance. This communicates with the physical
     * gyro hardware over CAN bus.
     */
    private final Pigeon2 gyro;

    /**
     * The odometry object that provides the most recently executed twist. This
     * is used to update the gyroscope orientation when the gyroscope is
     * disconnected or disabled.
     */
    private final Odometry odometry;

    // Status signal objects that contain the latest data from the gyroscope and
    // can be used to refresh the data
    private final StatusSignal<Angle> yawSignal;
    private final StatusSignal<AngularVelocity> yawVelocitySignal;
    private final StatusSignal<Angle> pitchSignal;
    private final StatusSignal<AngularVelocity> pitchVelocitySignal;
    private final StatusSignal<Angle> rollSignal;
    private final StatusSignal<AngularVelocity> rollVelocitySignal;

    // Cached orientation and angular velocity values. These may be based on
    // odometry instead of real gyroscope data depending on the configuration
    // and whether the gyroscope is disconnected.
    private Angle yaw = Radians.of(0);
    private AngularVelocity yawVelocity = RadiansPerSecond.of(0);
    private Angle pitch = Radians.of(0);
    private AngularVelocity pitchVelocity = RadiansPerSecond.of(0);
    private Angle roll = Radians.of(0);
    private AngularVelocity rollVelocity = RadiansPerSecond.of(0);

    /**
     * Constructs a Gyroscope object using the provided drivetrain constants.
     * 
     * @param constants The drivetrain constants containing gyroscope
     * configuration
     */
    public Gyroscope(DrivetrainConstants constants, Odometry odometry) {
        // Store the constants and Odometry
        this.constants = constants;
        this.odometry = odometry;

        // Create the Pigeon2 gyroscope instance
        gyro = new Pigeon2(constants.Gyroscope.CANId, constants.CANBusName);

        // Apply the Pigeon2Configuration in the DrivetrainConstants to the gyroscope
        StatusUtil.check(gyro.getConfigurator().apply(constants.Gyroscope.DeviceConfiguration));

        // Initialize the status signals without refreshing them
        yawSignal = gyro.getYaw(false);
        yawVelocitySignal = gyro.getAngularVelocityZWorld(false);
        pitchSignal = gyro.getPitch(false);
        pitchVelocitySignal = gyro.getAngularVelocityYWorld(false);
        rollSignal = gyro.getRoll(false);
        rollVelocitySignal = gyro.getAngularVelocityXWorld(false);
    }

    /**
     * Gets all status signals used by the gyroscope.
     */
    @Override
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
     * Returns true if the real gyroscope data should be used, and false if
     * odometry-derived data should be used instead.
     * 
     * @return True if real gyroscope data should be used
     */
    private boolean shouldUseGyroscope() {
        // To avoid accidentally disabling the gyroscope during a match, the
        // value of useRealGyro is ignored when connected to an FMS
        return (constants.Gyroscope.Enabled || DriverStation.isFMSAttached()) && gyro.isConnected();
    }

    @Override
    public void logTelemetry(String basePath) {
        basePath = LoggingUtil.ensureEndsWithSlash(basePath);

        DogLog.log(basePath + "IsConnected", gyro.isConnected());
        DogLog.log(basePath + "UsingGyroscope", shouldUseGyroscope());

        DogLog.log(basePath + "Yaw", getYaw().in(Radians), Radians);
        DogLog.log(basePath + "Pitch", getPitch().in(Radians), Radians);
        DogLog.log(basePath + "Roll", getRoll().in(Radians), Radians);

        DogLog.log(basePath + "YawVelocity", getYawVelocity().in(RadiansPerSecond), RadiansPerSecond);
        DogLog.log(basePath + "PitchVelocity", getPitchVelocity().in(RadiansPerSecond), RadiansPerSecond);
        DogLog.log(basePath + "RollVelocity", getRollVelocity().in(RadiansPerSecond), RadiansPerSecond);
    }

    /**
     * Updates the cached gyroscope data. This method should not be called while
     * the status signals are being refreshed in a different thread.
     */
    @Override
    public synchronized void update(double deltaTimeSeconds) {
        if (!shouldUseGyroscope()) {
            // If the gyroscope is disabled or disconnected, use odometry to
            // estimate the yaw angle and velocity. Pitch and roll are set to
            // zero, because odometry cannot be used to estimate them.
            Angle previousYaw = yaw;
            Angle deltaYaw = Radians.of(odometry.getTwist().dtheta);

            yaw = previousYaw.plus(deltaYaw);
            yawVelocity = deltaYaw.div(Seconds.of(deltaTimeSeconds));

            pitch = Radians.of(0);
            pitchVelocity = RadiansPerSecond.of(0);

            roll = Radians.of(0);
            rollVelocity = RadiansPerSecond.of(0);
        } else {
            // Copy the data stored in the status signals to the caching fields
            // to avoid a possible concurrency issue, where another thread
            // refreshes the signals while getYaw() or other getter methods are
            // called.
            if (constants.Gyroscope.LatencyCompensation) {
                yaw = Degrees.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                    yawSignal,
                    yawVelocitySignal
                ));

                pitch = Degrees.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                    pitchSignal,
                    pitchVelocitySignal
                ));

                roll = Degrees.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                    rollSignal,
                    rollVelocitySignal
                ));
            } else {
                yaw = yawSignal.getValue();
                pitch = pitchSignal.getValue();
                roll = rollSignal.getValue();
            }

            yawVelocity = yawVelocitySignal.getValue();
            pitchVelocity = pitchVelocitySignal.getValue();
            rollVelocity = rollVelocitySignal.getValue();
        }
    }

    @Override
    public void close() {
        gyro.close();
    }

    /**
     * Gets the yaw angle of the robot.
     * 
     * @return The yaw angle
     */
    public synchronized Angle getYaw() {
        return yaw;
    }

    /**
     * Gets the pitch angle of the robot.
     * 
     * @return The pitch angle
     */
    public synchronized Angle getPitch() {
        return pitch;
    }

    /**
     * Gets the roll angle of the robot.
     * 
     * @return The roll angle
     */
    public synchronized Angle getRoll() {
        return roll;
    }

    /**
     * Gets the yaw angular velocity of the robot.
     * 
     * @return The yaw angular velocity
     */
    public synchronized AngularVelocity getYawVelocity() {
        return yawVelocity;
    }

    /**
     * Gets the pitch angular velocity of the robot.
     * 
     * @return The pitch angular velocity
     */
    public synchronized AngularVelocity getPitchVelocity() {
        return pitchVelocity;
    }

    /**
     * Gets the roll angular velocity of the robot.
     * 
     * @return The roll angular velocity
     */
    public synchronized AngularVelocity getRollVelocity() {
        return rollVelocity;
    }
}
