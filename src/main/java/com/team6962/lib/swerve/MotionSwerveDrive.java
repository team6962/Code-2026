package com.team6962.lib.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;
import com.team6962.lib.swerve.localization.Gyroscope;
import com.team6962.lib.swerve.localization.Localization;
import com.team6962.lib.swerve.localization.Odometry;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.swerve.motion.LockMotion;
import com.team6962.lib.swerve.motion.NeutralMotion;
import com.team6962.lib.swerve.motion.SwerveMotion;
import com.team6962.lib.swerve.motion.SwerveMotionManager;
import com.team6962.lib.swerve.motion.VelocityMotion;
import com.team6962.lib.swerve.simulation.SwerveDriveSim;
import com.team6962.lib.swerve.util.ControlLoop;
import com.team6962.lib.swerve.util.FieldLogger;
import com.team6962.lib.swerve.util.SwerveComponent;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Core swerve drive implementation that manages motor control and localization.
 *
 * <p>The swerve drive can be controlled using {@link SwerveMotion} objects, which represent actions
 * that the robot can take during a single control loop cycle. The {@link #applyMotion
 * applyMotion()} method sets the next motion that the swerve drive will perform, or fuses a
 * translational and rotational motion together if one of the two has already been applied. The
 * {@link #updateMotion()} method begins executing the most recently applied motion, and prepares
 * the internal state for more motions to be applied in the next control loop cycle.
 *
 * <p>Various methods, including {@link #getPosition2d()}, {@link #getVelocity()}, and {@link
 * #getHeading()}, provide access to localization information about the state of the swerve drive.
 * The localization system fuses wheel motion data (odometry), gyroscope readings, and vision
 * measurements, which must be provided by external code using {@link #addVisionMeasurement
 * addVisionMeasurement()}.
 *
 * <p>When running in threaded mode, the swerve drive will be controlled using a higher frequency
 * control loop. Status signals, localization, telemetry, and swerve motions are updated inside of
 * the high frequency control loop, and thread safety is handled internally using {@code
 * synchronized} methods.
 *
 * <p>Physics simulation is supported and actives automatically when the code is running in
 * simulation.
 *
 * <p>When the swerve drive is no longer needed, make sure to call the {@link #close()} method to
 * release all resources and stop the control loop thread.
 *
 * @see SwerveModule
 * @see SwerveMotion
 * @see Localization
 * @see CommandSwerveDrive
 */
public class MotionSwerveDrive implements AutoCloseable {
  private DrivetrainConstants constants;
  private ControlLoop controlLoop;
  private SwerveModule[] modules;
  private Odometry odometry;
  private Gyroscope gyroscope;
  private Localization localization;
  private FieldLogger fieldLogger;
  private SwerveDriveSim simulation;
  private BaseStatusSignal[] statusSignals;
  private SwerveDriveKinematics kinematics;
  private SwerveMotionManager motionManager;

  /**
   * Creates a new MotionSwerveDrive with the specified drivetrain configuration.
   *
   * <p>This constructor initializes all swerve drive components, and the control loop is
   * automatically started and will begin updating odometry and executing motions immediately.
   *
   * @param constants The drivetrain configuration constants
   */
  public MotionSwerveDrive(DrivetrainConstants constants) {
    // Store the drivetrain constants for later use
    this.constants = constants;

    // Initialize the swerve modules, odometry, gyroscope, and localization
    // objects, which each handle a different aspect of the swerve
    // drivetrain's functionality
    modules = new SwerveModule[4];

    for (int i = 0; i < modules.length; i++) {
      modules[i] = new SwerveModule(Corner.fromIndex(i), constants);
    }

    odometry = new Odometry(constants, modules);
    gyroscope = new Gyroscope(constants, odometry);
    localization = new Localization(constants, new Pose3d(), odometry, gyroscope);

    // Initialize the kinematics object for converting between chassis
    // speeds and module states
    this.kinematics = constants.Structure.getKinematics();

    // Initialize the simulation, if we are running in simulation mode
    if (RobotBase.isSimulation()) {
      simulation = new SwerveDriveSim(constants, modules, gyroscope);
    }

    // Initialize the control loop, which will periodically call the
    // update() method, either in a thread or during the subsystem periodic
    controlLoop =
        constants.Timing.UseThreadedControlLoop
            ? new ControlLoop.Threaded()
            : new ControlLoop.SubsystemPeriodic();

    fieldLogger = new FieldLogger(constants, localization, odometry);

    // Build the array of status signals that will be refreshed in parallel
    SwerveComponent[] components = new SwerveComponent[3 + modules.length];

    components[0] = gyroscope;
    components[1] = odometry;
    components[2] = localization;

    for (int i = 0; i < modules.length; i++) {
      components[3 + i] = modules[i];
    }

    statusSignals = SwerveComponent.combineStatusSignals(components);

    BaseStatusSignal.setUpdateFrequencyForAll(constants.Timing.SignalUpdateRate, statusSignals);

    ParentDevice[] devices = SwerveComponent.combinePhoenixDevices(components);

    ParentDevice.optimizeBusUtilizationForAll(devices);

    motionManager = new SwerveMotionManager(new NeutralMotion(this));

    controlLoop.start(this::update, constants.Timing.ControlLoopFrequency);
  }

  private void update(double deltaTimeSeconds) {
    if (simulation != null) {
      simulation.update(deltaTimeSeconds);
    }

    BaseStatusSignal.refreshAll(statusSignals);

    // Compute and log the maximum latency of the status signals

    double signalLatencySeconds = 0.0;

    for (BaseStatusSignal signal : statusSignals) {
      double latency = signal.getTimestamp().getLatency();

      if (latency > signalLatencySeconds) {
        signalLatencySeconds = latency;
      }
    }

    DogLog.log("Drivetrain/SignalLatency", signalLatencySeconds);

    for (SwerveModule module : modules) {
      module.update(deltaTimeSeconds);
    }

    gyroscope.update(deltaTimeSeconds);
    odometry.update(deltaTimeSeconds);
    localization.update(deltaTimeSeconds);

    if (!constants.Simulation.EnablePoseEstimation && RobotBase.isSimulation()) {
      localization.resetPosition(simulation.getRobotPosition());
    }

    fieldLogger.update(deltaTimeSeconds);

    SwerveMotion motion = motionManager.getActiveMotion();

    if (motion != null) {
      motion.update(deltaTimeSeconds);
      motion.logTelemetry("Drivetrain/Motion");
    }

    for (SwerveModule module : modules) {
      module.logTelemetry("Drivetrain/Modules/" + module.getCorner().getName());
    }

    localization.logTelemetry("Drivetrain/Localization");
    fieldLogger.logTelemetry("Drivetrain/Field");
  }

  /**
   * Closes the swerve drive and releases all underlying resources. This should be called when the
   * swerve drive is no longer needed to ensure proper resource cleanup.
   */
  @Override
  public void close() {
    for (SwerveModule module : modules) {
      module.close();
    }

    gyroscope.close();
    motionManager.close();

    controlLoop.close();
  }

  /**
   * Gets the FieldLogger instance, which logs the robot's pose and module positions to a Field2d
   * widget on the dashboard for visualization.
   *
   * @return The FieldLogger instance used for logging robot and module poses to the dashboard
   */
  public FieldLogger getFieldLogger() {
    return fieldLogger;
  }

  /**
   * Gets the drivetrain constants used to configure this swerve drive's behavior.
   *
   * @return The drivetrain constants
   */
  public DrivetrainConstants getConstants() {
    return constants;
  }

  /**
   * Gets the swerve drive kinematics, which can be used to convert between chassis speeds and
   * swerve module states.
   *
   * @return The swerve drive kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Gets the array of swerve modules.
   *
   * <p>The modules are ordered by corner index: front-left (0), front-right (1), back-left (2),
   * back-right (3).
   *
   * @return The array of four swerve modules
   */
  public SwerveModule[] getModules() {
    return modules;
  }

  /**
   * Gets the {@link Odometry} object, which provides methods to get information relating to wheel
   * motions.
   *
   * @return The {@link Odometry} instance
   */
  public Odometry getOdometry() {
    return odometry;
  }

  /**
   * Gets the {@link Gyroscope} object, which provides methods for getting data from the physical
   * gyroscope on the robot.
   *
   * @return The {@link Gyroscope} instance
   */
  public Gyroscope getGyroscope() {
    return gyroscope;
  }

  /**
   * Gets the {@link Localization} object, which fuses odometry and vision data to estimate the
   * robot's position on the field.
   *
   * @return The {@link Localization} instance
   */
  public Localization getLocalization() {
    return localization;
  }

  /**
   * Gets the simulation instance, if running in simulation mode.
   *
   * @return The simulation instance, or {@code null} if not in simulation mode
   */
  public SwerveDriveSim getSimulation() {
    return simulation;
  }

  /**
   * Gets the current positions of all swerve modules, ordered by corner index: front-left (0),
   * front-right (1), back-left (2), back-right (3).
   *
   * <p>Module positions represent the cumulative distance traveled and current angle of each
   * module's wheel.
   *
   * @return Array of module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    return odometry.getPositions();
  }

  /**
   * Gets the change in module positions since the last update, ordered by corner index: front-left
   * (0), front-right (1), back-left (2), back-right (3).
   *
   * <p>Position deltas are used for odometry calculations to determine how far the robot has moved.
   *
   * @return Array of module position deltas
   */
  public SwerveModulePosition[] getModulePositionDeltas() {
    return odometry.getPositionDeltas();
  }

  /**
   * Gets the current states of all swerve modules, ordered by corner index: front-left (0),
   * front-right (1), back-left (2), back-right (3).
   *
   * <p>Module states include the current velocity and angle of each module.
   *
   * @return Array of module states
   */
  public SwerveModuleState[] getModuleStates() {
    return odometry.getStates();
  }

  /**
   * Gets the robot's twist (change in pose along an arc) since the last update.
   *
   * @return The twist representing dx, dy, and dtheta
   */
  public Twist2d getTwist() {
    return odometry.getTwist();
  }

  /**
   * Gets the current estimated position of the robot on the field.
   *
   * @return The robot's pose (x, y, rotation)
   */
  public Pose2d getPosition2d() {
    return localization.getPosition2d();
  }

  /**
   * Gets the current estimated 3D position of the robot on the field.
   *
   * @return The robot's 3D pose (x, y, z, rotation)
   */
  public Pose3d getPosition3d() {
    return localization.getPosition3d();
  }

  /**
   * Checks if the robot is within tolerances of a target pose.
   *
   * @param target The target pose to check against
   * @param translationTolerance Maximum allowed translation error from target position
   * @param angularTolerance Maximum allowed angular error from target rotation
   * @return {@code true} if within both tolerances, {@code false} otherwise
   */
  public boolean isNear(Pose2d target, Distance translationTolerance, Angle angularTolerance) {
    Pose2d current = getPosition2d();

    double linearError = current.getTranslation().getDistance(target.getTranslation());
    double angularError = Math.abs(current.getRotation().minus(target.getRotation()).getRadians());

    return linearError <= translationTolerance.in(Meters)
        && angularError <= angularTolerance.in(Radians);
  }

  /**
   * Gets the current field-relative velocity of the robot as a {@link ChassisSpeeds} object.
   *
   * @return The {@link ChassisSpeeds} (vx, vy, omega)
   */
  public ChassisSpeeds getVelocity() {
    return localization.getVelocity();
  }

  /**
   * Gets the current translational velocity of the robot.
   *
   * @return The {@link TranslationalVelocity} (x and y components)
   */
  public TranslationalVelocity getTranslationalVelocity() {
    return localization.getTranslationalVelocity();
  }

  /**
   * Gets the robot's velocity along an circular arc as a twist.
   *
   * <p>Arc velocity represents the twist the robot would perform if it continued to follow the
   * current circular arc for 1 second.
   *
   * @return The arc velocity as dx, dy, and dtheta per second
   */
  public Twist2d getArcVelocity() {
    return localization.getArcVelocity();
  }

  /**
   * Gets the robot's current heading (yaw angle).
   *
   * <p>This is an alias for {@link #getYaw()}.
   *
   * @return The heading angle
   */
  public Angle getHeading() {
    return localization.getHeading();
  }

  /**
   * Gets the robot's current yaw angle.
   *
   * <p>This is an alias for {@link #getHeading()}.
   *
   * @return The yaw angle
   */
  public Angle getYaw() {
    return getHeading();
  }

  /**
   * Gets the robot's angular velocity about the yaw axis.
   *
   * @return The rate of change of yaw
   */
  public AngularVelocity getYawVelocity() {
    return gyroscope.getYawVelocity();
  }

  /**
   * Gets the robot's current pitch angle (tilt forward/backward).
   *
   * @return The pitch angle
   */
  public Angle getPitch() {
    return gyroscope.getPitch();
  }

  /**
   * Gets the robot's angular velocity about the pitch axis.
   *
   * @return The rate of change of pitch
   */
  public AngularVelocity getPitchVelocity() {
    return gyroscope.getPitchVelocity();
  }

  /**
   * Gets the robot's current roll angle (tilt left/right).
   *
   * @return The roll angle
   */
  public Angle getRoll() {
    return gyroscope.getRoll();
  }

  /**
   * Gets the robot's angular velocity about the roll axis.
   *
   * @return The rate of change of roll
   */
  public AngularVelocity getRollVelocity() {
    return gyroscope.getRollVelocity();
  }

  /**
   * Begins executing the currently applied motion, and updates the motion manager's internal state
   * to prepare for a new motion to be applied.
   *
   * <p>This should be called periodically after motions are applied to ensure motion commands are
   * properly executed and motions from different control loop cycles are not fused together.
   */
  public void updateMotion() {
    motionManager.update();
  }

  /**
   * Applies a motion to be executed during the next control loop cycle.
   *
   * @param motion The motion to apply
   */
  public void applyMotion(SwerveMotion motion) {
    motionManager.apply(motion);
  }

  /**
   * Applies a velocity motion with the specified chassis speeds.
   *
   * <p>The robot will attempt to move at the given velocities during the next control loop cycle.
   *
   * @param velocity The desired chassis speeds
   */
  public void applyVelocityMotion(ChassisSpeeds velocity) {
    applyMotion(new VelocityMotion(velocity, this));
  }

  /**
   * Applies a velocity motion with separate x and y velocities (no rotation).
   *
   * <p>The robot will attempt to move at the given velocities during the next control loop cycle.
   *
   * @param xVelocity The x-component of velocity (forward/backward)
   * @param yVelocity The y-component of velocity (left/right)
   */
  public void applyVelocityMotion(LinearVelocity xVelocity, LinearVelocity yVelocity) {
    applyMotion(
        new VelocityMotion(
            new ChassisSpeeds(xVelocity.in(MetersPerSecond), yVelocity.in(MetersPerSecond), 0),
            this));
  }

  /**
   * Applies a velocity motion with a translational velocity (no rotation).
   *
   * <p>The robot will attempt to move at the given translational velocity during the next control
   * loop cycle.
   *
   * @param translationalVelocity The desired translational velocity
   */
  public void applyVelocityMotion(TranslationalVelocity translationalVelocity) {
    applyVelocityMotion(translationalVelocity.x, translationalVelocity.y);
  }

  /**
   * Applies a velocity motion with only angular velocity (rotation in place).
   *
   * <p>The robot will attempt to rotate at the given angular velocity during the next control loop
   * cycle.
   *
   * @param angularVelocity The desired angular velocity
   */
  public void applyVelocityMotion(AngularVelocity angularVelocity) {
    applyMotion(
        new VelocityMotion(new ChassisSpeeds(0, 0, angularVelocity.in(RadiansPerSecond)), this));
  }

  /**
   * Applies a neutral motion with the specified neutral mode. This motion will continue until the
   * next control loop cycle.
   *
   * <p>Neutral motion stops the robot and sets the motors to the specified neutral mode (coast or
   * brake).
   *
   * @param neutralMode The neutral mode for the motors
   */
  public void applyNeutralMotion(NeutralModeValue neutralMode) {
    applyMotion(new NeutralMotion(this, neutralMode));
  }

  /**
   * Applies a neutral motion with the default neutral mode. This motion will continue until the
   * next control loop cycle.
   *
   * <p>Neutral motion stops the robot and allows the motors to use their default neutral mode
   * behavior.
   */
  public void applyNeutralMotion() {
    applyMotion(new NeutralMotion(this, null));
  }

  /**
   * Applies a lock motion to prevent the robot from being pushed. This motion will continue until
   * the next control loop cycle.
   *
   * <p>Lock motion sets the swerve modules to an X pattern, which resists movement in all
   * directions. This is useful for defense or when stopped on an incline.
   */
  public void applyLockMotion() {
    applyMotion(new LockMotion(this));
  }
}
