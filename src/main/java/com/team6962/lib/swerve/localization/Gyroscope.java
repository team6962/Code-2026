package com.team6962.lib.swerve.localization;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.phoenix.StatusUtil;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.core.SwerveComponent;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Provides an interface to the Pigeon 2.0 gyroscope with methods to get
 * latency-compensated orientations and angular velocities.
 */
public class Gyroscope implements SwerveComponent {
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
     * True if the data from the real gyroscope device is being used, false if
     * using less accurate data derived from wheel odometry. This value will be
     * ignored when the real gyroscope is disconnected, and odometry-derived
     * data will be used.
     */
    private boolean useRealGyro;

    /**
     * The odometry object that provides the most recently executed twist.
     */
    private Odometry odometry;

    /**
     * The yaw angle derived from odometry data.
     */
    private Angle odometryDerivedYaw;

    /**
     * The yaw angular velocity derived from odometry data.
     */
    private AngularVelocity odometryDerivedYawVelocity;

    /**
     * Constructs a Gyroscope object using the provided drivetrain constants.
     * 
     * @param constants The drivetrain constants containing gyroscope
     * configuration
     */
    public Gyroscope(DrivetrainConstants constants, Odometry odometry) {
        this.constants = constants;

        useRealGyro = constants.Gyroscope.Enabled;

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
     * Sets whether the gyroscope should be enabled. When disabled, less
     * accurate data derived from wheel odometry will be used instead. When
     * connected to an FMS, the gyroscope will always be enabled regardless of
     * this setting.
     * 
     * @param enabled True to enable the gyroscope, false to disable it
     */
    public void setEnabled(boolean enabled) {
        useRealGyro = enabled;
    }

    /**
     * Returns true if the real gyroscope data should be used, and false if
     * odometry-derived data should be used instead.
     * 
     * @return True if real gyroscope data should be used
     */
    private boolean shouldUseRealGyroscope() {
        // To avoid accidentally disabling the gyroscope during a match, the
        // value of useRealGyro is ignored when connected to an FMS
        return (useRealGyro || DriverStation.isFMSAttached()) && gyro.isConnected();
    }

    /**
     * Gets the yaw angle of the robot.
     * 
     * @return The yaw angle
     */
    public Angle getYaw() {
        if (!shouldUseRealGyroscope()) {
            return odometryDerivedYaw;
        } else if (constants.Gyroscope.LatencyCompensation) {
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
        if (!shouldUseRealGyroscope()) {
            return Radians.zero();
        } else if (constants.Gyroscope.LatencyCompensation) {
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
        if (!shouldUseRealGyroscope()) {
            return Radians.zero();
        } else if (constants.Gyroscope.LatencyCompensation) {
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
        if (shouldUseRealGyroscope()) {
            return yawVelocitySignal.getValue();
        } else {
            return odometryDerivedYawVelocity;
        }
    }

    /**
     * Gets the pitch angular velocity of the robot.
     * 
     * @return The pitch angular velocity
     */
    public AngularVelocity getPitchVelocity() {
        if (shouldUseRealGyroscope()) {
            return pitchVelocitySignal.getValue();
        } else {
            return RadiansPerSecond.zero();
        }
    }

    /**
     * Gets the roll angular velocity of the robot.
     * 
     * @return The roll angular velocity
     */
    public AngularVelocity getRollVelocity() {
        if (shouldUseRealGyroscope()) {
            return rollVelocitySignal.getValue();
        } else {
            return RadiansPerSecond.zero();
        }
    }

    @Override
    public void logTelemetry(String basePath) {
        basePath = LoggingUtil.ensureEndsWithSlash(basePath);

        DogLog.log(basePath + "UsingRealGyro", shouldUseRealGyroscope());

        DogLog.log(basePath + "Yaw", getYaw().in(Radians), Radians);
        DogLog.log(basePath + "Pitch", getPitch().in(Radians), Radians);
        DogLog.log(basePath + "Roll", getRoll().in(Radians), Radians);

        DogLog.log(basePath + "YawVelocity", getYawVelocity().in(RadiansPerSecond), RadiansPerSecond);
        DogLog.log(basePath + "PitchVelocity", getPitchVelocity().in(RadiansPerSecond), RadiansPerSecond);
        DogLog.log(basePath + "RollVelocity", getRollVelocity().in(RadiansPerSecond), RadiansPerSecond);
    }

    @Override
    public void update(double deltaTimeSeconds) {
        Angle previousYaw = odometryDerivedYaw;
        Angle deltaYaw = Radians.of(odometry.getTwist().dtheta);

        odometryDerivedYaw = previousYaw.plus(deltaYaw);
        odometryDerivedYawVelocity = deltaYaw.div(Seconds.of(deltaTimeSeconds));
    }
}
