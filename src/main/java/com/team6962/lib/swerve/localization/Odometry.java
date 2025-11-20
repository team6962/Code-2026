package com.team6962.lib.swerve.localization;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.core.SwerveComponent;
import com.team6962.lib.swerve.module.SwerveModule;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Odometry class for tracking the positions and states of swerve modules and
 * the changes in position of the robot over time.
 */
public class Odometry implements SwerveComponent {
    /**
     * Kinematics object for converting between module states/positions and
     * robot motion.
     */
    private SwerveDriveKinematics kinematics;

    /**
     * Array of swerve modules being tracked by the odometry.
     */
    private SwerveModule[] modules;

    /**
     * The last recorded positions of the swerve modules.
     */
    private SwerveModulePosition[] lastPositions;

    /**
     * The current recorded positions of the swerve modules.
     */
    private SwerveModulePosition[] currentPostions;

    /**
     * The current recorded states of the swerve modules.
     */
    private SwerveModuleState[] currentStates;

    /**
     * The change in position of the swerve modules since the last update.
     */
    private SwerveModulePosition[] positionDeltas;

    /**
     * The twist the robot has undergone since the last update.
     */
    private Twist2d twist = new Twist2d();

    /**
     * Constructs a new Odometry object with the specified kinematics and swerve
     * modules.
     * 
     * @param constants The DrivetrainConstants object for the swerve drivetrain
     * @param modules   An array of SwerveModule objects representing the swerve
     * modules being tracked.
     */
    public Odometry(DrivetrainConstants constants, SwerveModule[] modules) {
        this.kinematics = constants.Structure.getKinematics();
        this.modules = modules;
        lastPositions = computePositions();
        currentPostions = computePositions();
        currentStates = computeStates();
    }

    /**
     * Computes the current positions (steer angles and drive positions) of all
     * swerve modules. This method should only be called internally to refresh
     * the position data.
     * 
     * @return An array of SwerveModulePosition objects representing the current
     * positions of the swerve modules.
     */
    private SwerveModulePosition[] computePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }

        return positions;
    }

    /**
     * Gets the current positions of all swerve modules. These include the
     * distances each wheel has traveled along the ground and the current
     * direction that each wheel is facing.
     * 
     * @return An array of SwerveModulePosition objects representing the current
     * positions of the swerve modules.
     */
    public synchronized SwerveModulePosition[] getPositions() {
        return currentPostions;
    }

    /**
     * Computes the current states (steer angles and drive velocities) of all
     * swerve modules. This method should only be called internally to refresh
     * the state data.
     * 
     * @return An array of SwerveModuleState objects representing the current
     */
    private SwerveModuleState[] computeStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }

        return states;
    }

    /**
     * Gets the current states of all swerve modules. These include the linear
     * velocities and steer angles of each wheel.
     * 
     * @return An array of SwerveModuleState objects representing the current
     * states of the swerve modules.
     */
    public synchronized SwerveModuleState[] getStates() {
        return currentStates;
    }

    /**
     * Computes the change in position of all swerve modules since the last
     * update. This method should only be called internally to refresh the
     * position delta data.
     * 
     * @return An array of SwerveModulePosition objects representing the change
     * in position of the swerve modules.
     */
    private SwerveModulePosition[] computePositionDeltas() {
        SwerveModulePosition[] deltas = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            double deltaDistance = currentPostions[i].distanceMeters - lastPositions[i].distanceMeters;
            deltas[i] = new SwerveModulePosition(deltaDistance, currentPostions[i].angle);
        }
        return deltas;
    }

    /**
     * Gets the change in position of all swerve modules since the last update.
     * 
     * @return An array of SwerveModulePosition objects representing the change
     * in position of the swerve modules.
     */
    public synchronized SwerveModulePosition[] getPositionDeltas() {
        return positionDeltas;
    }

    /**
     * Computes the twist the robot has undergone since the last update. This
     * method should only be called internally to refresh the twist data.
     * 
     * @return A Twist2d object representing the change in position and
     * orientation of the robot.
     */
    private Twist2d computeTwist() {
        return kinematics.toTwist2d(positionDeltas);
    }

    /**
     * Gets the twist the robot has undergone since the last update.
     * 
     * @return A Twist2d object representing the change in position and
     * orientation of the robot.
     */
    public synchronized Twist2d getTwist() {
        return twist;
    }

    @Override
    public synchronized void logTelemetry(String basePath) {
        basePath = LoggingUtil.ensureEndsWithSlash(basePath);

        for (int i = 0; i < modules.length; i++) {
            DogLog.log(basePath + "Module Positions/" + i + "/distanceMeters", currentPostions[i].distanceMeters, Meters);
            DogLog.log(basePath + "Module Positions/" + i + "/angleRadians", currentPostions[i].angle.getRadians(), Radians);
            DogLog.log(basePath + "Module States/" + i + "/speedMetersPerSecond", currentStates[i].speedMetersPerSecond, MetersPerSecond);
            DogLog.log(basePath + "Module States/" + i + "/angleRadians", currentStates[i].angle.getRadians(), Radians);
            DogLog.log(basePath + "Position Deltas/" + i + "/distanceMeters", positionDeltas[i].distanceMeters, Meters);
            DogLog.log(basePath + "Position Deltas/" + i + "/angleRadians", positionDeltas[i].angle.getRadians(), Radians);
        }

        DogLog.log(basePath + "Twist/dx", twist.dx, Meters);
        DogLog.log(basePath + "Twist/dy", twist.dy, Meters);
        DogLog.log(basePath + "Twist/dtheta", twist.dtheta, Radians);
    }

    @Override
    public synchronized void update(double deltaTimeSeconds) {
        lastPositions = currentPostions;
        currentPostions = computePositions();
        currentStates = computeStates();
        positionDeltas = computePositionDeltas();
        twist = computeTwist();
    }
}
