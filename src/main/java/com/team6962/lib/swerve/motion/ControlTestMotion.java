package com.team6962.lib.swerve.motion;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.team6962.lib.swerve.MotionSwerveDrive;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;

import dev.doglog.DogLog;

/**
 * A swerve motion for running test control requests on motor controllers.
 * 
 * <p>ControlTestMotion allows direct, low-level control of individual swerve
 * module motors by sending arbitrary {@link ControlRequest} objects to specific
 * drive or steer motors. This is primarily useful for:
 * <ul>
 *   <li>Testing and tuning individual motor controllers</li>
 *   <li>Debugging motor behavior in isolation</li>
 *   <li>Characterization routines (SysId, etc.)</li>
 *   <li>Manual motor control during development</li>
 * </ul>
 * 
 * <p>The class provides a builder API with {@code with*()} methods for
 * convenient configuration, as well as direct setter methods. Control requests
 * are stored in an array where even indices (0, 2, 4, 6) are drive motors and
 * odd indices (1, 3, 5, 7) are steer motors.
 * 
 * @see SwerveMotion
 * @see ControlRequest
 */
public class ControlTestMotion implements SwerveMotion {
    /**
     * Array of control requests: [ front left drive, front left steer, front
     * right drive, front right steer, back left drive, back left steer, back
     * right drive, back right steer].
     */
    private ControlRequest[] controlRequests;
    
    /** The swerve drive this motion controls. */
    private MotionSwerveDrive swerveDrive;

    /**
     * Creates a new ControlTestMotion with the specified control requests.
     * 
     * @param swerveDrive The swerve drive to control
     * @param controlRequests Array of control requests (2 per module: drive, steer)
     */
    public ControlTestMotion(MotionSwerveDrive swerveDrive, ControlRequest[] controlRequests) {
        this.swerveDrive = swerveDrive;
        this.controlRequests = controlRequests;
    }

    /**
     * Creates a new ControlTestMotion with all motors set to neutral.
     * 
     * @param swerveDrive The swerve drive to control
     */
    public ControlTestMotion(MotionSwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        int moduleCount = swerveDrive.getModules().length;
        this.controlRequests = new ControlRequest[moduleCount * 2];

        NeutralOut neutralOut = new NeutralOut()
            .withUpdateFreqHz(swerveDrive.getConstants().Timing.ControlLoopFrequency.in(Hertz))
            .withUseTimesync(swerveDrive.getConstants().Timing.TimesyncControlRequests);

        for (int i = 0; i < moduleCount; i++) {
            this.controlRequests[2 * i] = neutralOut;
            this.controlRequests[2 * i + 1] = neutralOut;
        }
    }

    /**
     * Sets the drive motor control request for a specific module.
     * 
     * @param moduleCorner The corner of the module to control
     * @param driveRequest The control request for the drive motor
     */
    public synchronized void setDriveControl(Corner moduleCorner, ControlRequest driveRequest) {
        int moduleIndex = moduleCorner.getIndex();
        controlRequests[2 * moduleIndex] = driveRequest;
    }

    /**
     * Sets the steer motor control request for a specific module.
     * 
     * @param moduleCorner The corner of the module to control
     * @param steerRequest The control request for the steer motor
     */
    public synchronized void setSteerControl(Corner moduleCorner, ControlRequest steerRequest) {
        int moduleIndex = moduleCorner.getIndex();
        controlRequests[2 * moduleIndex + 1] = steerRequest;
    }

    /**
     * Sets both drive and steer motor control requests for a specific module.
     * 
     * @param moduleCorner The corner of the module to control
     * @param driveRequest The control request for the drive motor
     * @param steerRequest The control request for the steer motor
     */
    public synchronized void setModuleControl(Corner moduleCorner, ControlRequest driveRequest, ControlRequest steerRequest) {
        int moduleIndex = moduleCorner.getIndex();
        controlRequests[2 * moduleIndex] = driveRequest;
        controlRequests[2 * moduleIndex + 1] = steerRequest;
    }

    /**
     * Sets the same drive control request for all modules.
     * 
     * @param driveRequest The control request to apply to all drive motors
     */
    public void setAllDriveControl(ControlRequest driveRequest) {
        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            setDriveControl(Corner.fromIndex(i), driveRequest);
        }
    }

    /**
     * Sets the same steer control request for all modules.
     * 
     * @param steerRequest The control request to apply to all steer motors
     */
    public void setAllSteerControl(ControlRequest steerRequest) {
        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            setSteerControl(Corner.fromIndex(i), steerRequest);
        }
    }

    /**
     * Sets the same drive and steer control requests for all modules.
     * 
     * @param driveRequest The control request to apply to all drive motors
     * @param steerRequest The control request to apply to all steer motors
     */
    public void setAllModuleControl(ControlRequest driveRequest, ControlRequest steerRequest) {
        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            setModuleControl(Corner.fromIndex(i), driveRequest, steerRequest);
        }
    }

    /**
     * Sets the drive control for the front-left module.
     * 
     * @param driveRequest The control request for the drive motor
     */
    public void setFrontLeftDriveControl(ControlRequest driveRequest) {
        setDriveControl(Corner.FrontLeft, driveRequest);
    }

    /**
     * Sets the drive control for the front-right module.
     * 
     * @param driveRequest The control request for the drive motor
     */
    public void setFrontRightDriveControl(ControlRequest driveRequest) {
        setDriveControl(Corner.FrontRight, driveRequest);
    }

    /**
     * Sets the drive control for the back-left module.
     * 
     * @param driveRequest The control request for the drive motor
     */
    public void setBackLeftDriveControl(ControlRequest driveRequest) {
        setDriveControl(Corner.BackLeft, driveRequest);
    }

    /**
     * Sets the drive control for the back-right module.
     * 
     * @param driveRequest The control request for the drive motor
     */
    public void setBackRightDriveControl(ControlRequest driveRequest) {
        setDriveControl(Corner.BackRight, driveRequest);
    }

    /**
     * Sets the steer control for the front-left module.
     * 
     * @param steerRequest The control request for the steer motor
     */
    public void setFrontLeftSteerControl(ControlRequest steerRequest) {
        setSteerControl(Corner.FrontLeft, steerRequest);
    }

    /**
     * Sets the steer control for the front-right module.
     * 
     * @param steerRequest The control request for the steer motor
     */
    public void setFrontRightSteerControl(ControlRequest steerRequest) {
        setSteerControl(Corner.FrontRight, steerRequest);
    }

    /**
     * Sets the steer control for the back-left module.
     * 
     * @param steerRequest The control request for the steer motor
     */
    public void setBackLeftSteerControl(ControlRequest steerRequest) {
        setSteerControl(Corner.BackLeft, steerRequest);
    }

    /**
     * Sets the steer control for the back-right module.
     * 
     * @param steerRequest The control request for the steer motor
     */
    public void setBackRightSteerControl(ControlRequest steerRequest) {
        setSteerControl(Corner.BackRight, steerRequest);
    }

    /**
     * Sets the drive control for a specific module and returns this for chaining.
     * 
     * @param moduleCorner The corner of the module to control
     * @param driveRequest The control request for the drive motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withDriveControl(Corner moduleCorner, ControlRequest driveRequest) {
        setDriveControl(moduleCorner, driveRequest);
        return this;
    }

    /**
     * Sets the steer control for a specific module and returns this for chaining.
     * 
     * @param moduleCorner The corner of the module to control
     * @param steerRequest The control request for the steer motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withSteerControl(Corner moduleCorner, ControlRequest steerRequest) {
        setSteerControl(moduleCorner, steerRequest);
        return this;
    }

    /**
     * Sets both controls for a specific module and returns this for chaining.
     * 
     * @param moduleCorner The corner of the module to control
     * @param driveRequest The control request for the drive motor
     * @param steerRequest The control request for the steer motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withModuleControl(Corner moduleCorner, ControlRequest driveRequest, ControlRequest steerRequest) {
        setModuleControl(moduleCorner, driveRequest, steerRequest);
        return this;
    }

    /**
     * Sets the drive control for all modules and returns this for chaining.
     * 
     * @param driveRequest The control request to apply to all drive motors
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withAllDriveControl(ControlRequest driveRequest) {
        setAllDriveControl(driveRequest);
        return this;
    }

    /**
     * Sets the steer control for all modules and returns this for chaining.
     * 
     * @param steerRequest The control request to apply to all steer motors
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withAllSteerControl(ControlRequest steerRequest) {
        setAllSteerControl(steerRequest);
        return this;
    }

    /**
     * Sets both controls for all modules and returns this for chaining.
     * 
     * @param driveRequest The control request to apply to all drive motors
     * @param steerRequest The control request to apply to all steer motors
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withAllModuleControl(ControlRequest driveRequest, ControlRequest steerRequest) {
        setAllModuleControl(driveRequest, steerRequest);
        return this;
    }

    /**
     * Sets the front-left drive control and returns this for chaining.
     * 
     * @param driveRequest The control request for the drive motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withFrontLeftDriveControl(ControlRequest driveRequest) {
        setFrontLeftDriveControl(driveRequest);
        return this;
    }

    /**
     * Sets the front-right drive control and returns this for chaining.
     * 
     * @param driveRequest The control request for the drive motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withFrontRightDriveControl(ControlRequest driveRequest) {
        setFrontRightDriveControl(driveRequest);
        return this;
    }

    /**
     * Sets the back-left drive control and returns this for chaining.
     * 
     * @param driveRequest The control request for the drive motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withBackLeftDriveControl(ControlRequest driveRequest) {
        setBackLeftDriveControl(driveRequest);
        return this;
    }

    /**
     * Sets the back-right drive control and returns this for chaining.
     * 
     * @param driveRequest The control request for the drive motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withBackRightDriveControl(ControlRequest driveRequest) {
        setBackRightDriveControl(driveRequest);
        return this;
    }

    /**
     * Sets the front-left steer control and returns this for chaining.
     * 
     * @param steerRequest The control request for the steer motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withFrontLeftSteerControl(ControlRequest steerRequest) {
        setFrontLeftSteerControl(steerRequest);
        return this;
    }

    /**
     * Sets the front-right steer control and returns this for chaining.
     * 
     * @param steerRequest The control request for the steer motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withFrontRightSteerControl(ControlRequest steerRequest) {
        setFrontRightSteerControl(steerRequest);
        return this;
    }

    /**
     * Sets the back-left steer control and returns this for chaining.
     * 
     * @param steerRequest The control request for the steer motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withBackLeftSteerControl(ControlRequest steerRequest) {
        setBackLeftSteerControl(steerRequest);
        return this;
    }

    /**
     * Sets the back-right steer control and returns this for chaining.
     * 
     * @param steerRequest The control request for the steer motor
     * @return This ControlTestMotion for method chaining
     */
    public ControlTestMotion withBackRightSteerControl(ControlRequest steerRequest) {
        setBackRightSteerControl(steerRequest);
        return this;
    }

    /**
     * Applies all configured control requests to the swerve modules.
     * 
     * <p>This method sends each stored control request to its corresponding motor
     * on every control loop iteration.
     * 
     * @param deltaTimeSeconds The time since the last update
     */
    @Override
    public synchronized void update(double deltaTimeSeconds) {
        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            swerveDrive.getModules()[i].setControl(controlRequests[2 * i], controlRequests[2 * i + 1]);
        }
    }

    /**
     * Logs telemetry data for this control test motion.
     * 
     * @param basePath The base path for telemetry logging
     */
    @Override
    public synchronized void logTelemetry(String basePath) {
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }
        
        DogLog.log(basePath + "Type", "ControlTest");
    }
}
