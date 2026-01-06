package com.team6962.lib.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotionSwerveDrive extends SubsystemBase implements AutoCloseable {
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
        localization = new Localization(constants, new Pose2d(), odometry, gyroscope);

        // Initialize the kinematics object for converting between chassis
        // speeds and module states
        this.kinematics = constants.Structure.getKinematics();

        // Initialize the simulation, if we are running in simulation mode
        if (RobotBase.isSimulation()) {
            simulation = new SwerveDriveSim(constants, modules, gyroscope);
        }

        // Initialize the control loop, which will periodically call the
        // update() method, either in a thread or during the subsystem periodic
        controlLoop = constants.Timing.UseThreadedControlLoop ?
            new ControlLoop.Threaded() :
            new ControlLoop.SubsystemPeriodic();

        fieldLogger = new FieldLogger(constants, this::getPosition, this::getModuleStates);

        // Build the array of status signals that will be refreshed in parallel
        SwerveComponent[] components = new SwerveComponent[3 + modules.length];

        components[0] = gyroscope;
        components[1] = odometry;
        components[2] = localization;

        for (int i = 0; i < modules.length; i++) {
            components[3 + i] = modules[i];
        }

        statusSignals = SwerveComponent.combineStatusSignals(components);

        motionManager = new SwerveMotionManager(new NeutralMotion(this));

        controlLoop.start(this::update, constants.Timing.ControlLoopFrequency);
    }

    private void update(double deltaTimeSeconds) {
        if (simulation != null) {
            simulation.update(deltaTimeSeconds);
        }

        if (constants.Timing.UseThreadedControlLoop) {
            BaseStatusSignal.refreshAll(statusSignals);
        }

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
        fieldLogger.update(deltaTimeSeconds);

        SwerveMotion motion = motionManager.getActiveMotion();

        if (motion != null) {
            motion.update(deltaTimeSeconds);
        }

        for (SwerveModule module : modules) {
            module.logTelemetry("Drivetrain/Modules/" + module.getCorner().getName());
        }

        localization.logTelemetry("Drivetrain/Localization");
        fieldLogger.logTelemetry("Drivetrain/Field");

        if (!constants.Timing.UseThreadedControlLoop) {
            BaseStatusSignal.refreshAll(statusSignals);
        }
    }

    @Override
    public void close() {
        for (SwerveModule module : modules) {
            module.close();
        }

        gyroscope.close();
        motionManager.close();

        controlLoop.close();
    }

    public DrivetrainConstants getConstants() {
        return constants;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public Odometry getOdometry() {
        return odometry;
    }

    public Gyroscope getGyroscope() {
        return gyroscope;
    }

    public Localization getLocalization() {
        return localization;
    }

    public SwerveDriveSim getSimulation() {
        return simulation;
    }

    public SwerveModulePosition[] getModulePositions() {
        return odometry.getPositions();
    }

    public SwerveModulePosition[] getModulePositionDeltas() {
        return odometry.getPositionDeltas();
    }

    public SwerveModuleState[] getModuleStates() {
        return odometry.getStates();
    }

    public Twist2d getTwist() {
        return odometry.getTwist();
    }

    public Pose2d getPosition() {
        return localization.getPosition();
    }

    public boolean isNear(Pose2d target, Distance linearTolerance, Angle angularTolerance) {
        Pose2d current = getPosition();

        double linearError = current.getTranslation().getDistance(target.getTranslation());
        double angularError = Math.abs(current.getRotation().minus(target.getRotation()).getRadians());

        return linearError <= linearTolerance.in(Meters) &&
               angularError <= angularTolerance.in(Radians);
    }

    public ChassisSpeeds getVelocity() {
        return localization.getVelocity();
    }

    public TranslationalVelocity getTranslationalVelocity() {
        return localization.getTranslationalVelocity();
    }

    public Twist2d getArcVelocity() {
        return localization.getArcVelocity();
    }

    public void addVisionMeasurement(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ) {
        localization.addVisionEstimate(pose, timestampSeconds, stdDevs);
    }

    public void addVisionEstimate(
        Pose2d pose,
        double timestampSeconds
    ) {
        localization.addVisionEstimate(pose, timestampSeconds);
    }

    public Angle getHeading() {
        return localization.getHeading();
    }

    public Angle getYaw() {
        return getHeading();
    }

    public AngularVelocity getYawVelocity() {
        return gyroscope.getYawVelocity();
    }

    public Angle getPitch() {
        return gyroscope.getPitch();
    }

    public AngularVelocity getPitchVelocity() {
        return gyroscope.getPitchVelocity();
    }

    public Angle getRoll() {
        return gyroscope.getRoll();
    }

    public AngularVelocity getRollVelocity() {
        return gyroscope.getRollVelocity();
    }

    public void clearMotion() {
        motionManager.update();
    }

    public void applyMotion(SwerveMotion motion) {
        motionManager.apply(motion);
    }

    public void applyVelocityMotion(ChassisSpeeds velocity) {
        applyMotion(new VelocityMotion(velocity, this));
    }

    public void applyVelocityMotion(LinearVelocity xVelocity, LinearVelocity yVelocity) {
        applyMotion(new VelocityMotion(new ChassisSpeeds(
            xVelocity.in(MetersPerSecond),
            yVelocity.in(MetersPerSecond),
            0
        ), this));
    }

    public void applyVelocityMotion(TranslationalVelocity translationalVelocity) {
        applyVelocityMotion(translationalVelocity.x, translationalVelocity.y);
    }

    public void applyVelocityMotion(AngularVelocity angularVelocity) {
        applyMotion(new VelocityMotion(new ChassisSpeeds(
            0,
            0,
            angularVelocity.in(RadiansPerSecond)
        ), this));
    }

    public void applyNeutralMotion(NeutralModeValue neutralMode) {
        applyMotion(new NeutralMotion(this, neutralMode));
    }

    public void applyNeutralMotion() {
        applyMotion(new NeutralMotion(this, null));
    }

    public void applyLockMotion() {
        applyMotion(new LockMotion(this));
    }
}
