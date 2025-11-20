package com.team6962.lib.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team6962.lib.math.TranslationalVelocity;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;
import com.team6962.lib.swerve.core.SwerveComponent;
import com.team6962.lib.swerve.core.SwerveControlLoop;
import com.team6962.lib.swerve.localization.Gyroscope;
import com.team6962.lib.swerve.localization.Localization;
import com.team6962.lib.swerve.localization.Odometry;
import com.team6962.lib.swerve.module.SwerveModule;
import com.team6962.lib.swerve.motion.SwerveMotion;
import com.team6962.lib.swerve.motion.SwerveMotionManager;
import com.team6962.lib.swerve.motion.VelocityMotion;
import com.team6962.lib.swerve.simulation.SwerveDriveSim;

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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

public class MotionSwerveDrive implements AutoCloseable {
    private DrivetrainConstants constants;
    private SwerveControlLoop controlLoop;
    private SwerveModule[] modules;
    private Odometry odometry;
    private Gyroscope gyroscope;
    private Localization localization;
    private SwerveDriveSim simulation;
    private BaseStatusSignal[] statusSignals;
    private SwerveDriveKinematics kinematics;
    private SwerveMotionManager motionManager = new SwerveMotionManager();

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
        localization = new Localization(constants, new Pose2d(), odometry);

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
            new SwerveControlLoop.Threaded() :
            new SwerveControlLoop.SubsystemPeriodic();

        controlLoop.start(this::update, constants.Timing.ControlLoopFrequency);

        // Build the array of status signals that will be refreshed in parallel
        SwerveComponent[] components = new SwerveComponent[3 + modules.length];

        components[0] = gyroscope;
        components[1] = odometry;
        components[2] = localization;

        for (int i = 0; i < modules.length; i++) {
            components[3 + i] = modules[i];
        }

        statusSignals = SwerveComponent.combineStatusSignals(components);
    }

    private void update(double deltaTimeSeconds) {
        if (simulation != null) {
            simulation.update(deltaTimeSeconds);
        }

        if (constants.Timing.UseThreadedControlLoop) {
            BaseStatusSignal.waitForAll(
                constants.Timing.SignalUpdateRate.asPeriod().in(Seconds),
                statusSignals
            );
        }

        for (SwerveModule module : modules) {
            module.update(deltaTimeSeconds);
        }

        gyroscope.update(deltaTimeSeconds);
        odometry.update(deltaTimeSeconds);
        localization.update(deltaTimeSeconds);

        SwerveMotion motion = motionManager.getMotion();

        if (motion != null) {
            motion.update(deltaTimeSeconds);
        }

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

    public ChassisSpeeds getVelocity() {
        return localization.getVelocity();
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
        motionManager.clear();
    }

    public void applyMotion(SwerveMotion motion) {
        motionManager.apply(motion);
    }

    public void applyVelocityMotion(ChassisSpeeds velocity) {
        applyMotion(new VelocityMotion(velocity, true, true, this));
    }

    public void applyVelocityMotion(LinearVelocity xVelocity, LinearVelocity yVelocity) {
        applyMotion(new VelocityMotion(new ChassisSpeeds(
            xVelocity.in(MetersPerSecond),
            yVelocity.in(MetersPerSecond),
            0
        ), true, false, this));
    }

    public void applyVelocityMotion(TranslationalVelocity translationalVelocity) {
        applyVelocityMotion(translationalVelocity.x, translationalVelocity.y);
    }

    public void applyVelocityMotion(AngularVelocity angularVelocity) {
        applyMotion(new VelocityMotion(new ChassisSpeeds(
            0,
            0,
            angularVelocity.in(RadiansPerSecond)
        ), false, true, this));
    }
}
