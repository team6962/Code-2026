package com.team6962.lib.swerve.config;

import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;

import edu.wpi.first.units.measure.Distance;

/**
 * A set of constants that configure the behavior of a swerve drivetrain.
 */
public class DrivetrainConstants {
    /**
     * The name of the CAN bus that the drivetrain's devices are connected to.
     * This defaults to an empty string, which means the RIO CAN bus on a
     * RoboRIO. If your drivetrain is using a different CAN bus, set this
     * to the name of the canivore.
     */
    public String CANBusName = "";

    /**
     * The constants that define the behavior of the gyroscope used for
     * determining the heading of the robot.
     */
    public GyroscopeConstants Gyroscope = new GyroscopeConstants();

    /**
     * The constants that define various update frequencies and whether to use
     * timesync for control requests.
     */
    public TimingConstants Timing = new TimingConstants();

    /**
     * The constants that define the drive motor behavior.
     */
    public DriveMotorConstants DriveMotor = new DriveMotorConstants();

    /**
     * The constants that define the steer motor behavior.
     */
    public SteerMotorConstants SteerMotor = new SteerMotorConstants();

    /**
     * The constants that define the steer encoder behavior.
     */
    public SteerEncoderConstants SteerEncoder = new SteerEncoderConstants();

    /**
     * The constants for each of the swerve modules in the drivetrain, which
     * include the CAN IDs of each device, absolute encoder offsets, and
     * optional wheel radii.
     */
    public SwerveModuleConstants[] SwerveModules = new SwerveModuleConstants[4];

    /**
     * The constants that define the driving behavior of the drivetrain.
     */
    public DrivingConstants Driving = new DrivingConstants();

    /**
     * The constants that define the physical structure of the drivetrain.
     */
    public StructureConstants Structure = new StructureConstants();

    public DrivetrainConstants() {
    }

    /**
     * Sets the name of the CAN bus that the drivetrain's devices are connected
     * to, and returns this DrivetrainConstants for chaining.
     * 
     * @param canBusName The name of the CAN bus
     * @return           This DrivetrainConstants object
     */
    public DrivetrainConstants withCANBusName(String canBusName) {
        CANBusName = canBusName;
        return this;
    }

    /**
     * Sets the structure constants for the drivetrain, and returns this
     * DrivetrainConstants for chaining.
     * 
     * @param structure The structure constants
     * @return          This DrivetrainConstants object
     */
    public DrivetrainConstants withStructure(StructureConstants structure) {
        Structure = structure;
        return this;
    }

    /**
     * Sets the gyroscope constants for the drivetrain, and returns this
     * DrivetrainConstants for chaining.
     * 
     * @param gyroscope The gyroscope constants
     * @return          This DrivetrainConstants object
     */
    public DrivetrainConstants withGyroscope(GyroscopeConstants gyroscope) {
        Gyroscope = gyroscope;
        return this;
    }

    /**
     * Sets the timing constants for the drivetrain, and returns this
     * DrivetrainConstants for chaining.
     * 
     * @param timing The timing constants
     * @return       This DrivetrainConstants object
     */
    public DrivetrainConstants withTiming(TimingConstants timing) {
        Timing = timing;
        return this;
    }

    /**
     * Sets the drive motor constants for the drivetrain, and returns this
     * DrivetrainConstants for chaining.
     * 
     * @param driveMotor The drive motor constants
     * @return           This DrivetrainConstants object
     */
    public DrivetrainConstants withDriveMotor(DriveMotorConstants driveMotor) {
        DriveMotor = driveMotor;
        return this;
    }

    /**
     * Sets the steer motor constants for the drivetrain, and returns this
     * DrivetrainConstants for chaining.
     * 
     * @param steerMotor The steer motor constants
     * @return           This DrivetrainConstants object
     */
    public DrivetrainConstants withSteerMotor(SteerMotorConstants steerMotor) {
        SteerMotor = steerMotor;
        return this;
    }

    /**
     * Sets the steer encoder constants for the drivetrain, and returns this
     * DrivetrainConstants for chaining.
     * 
     * @param steerEncoder The steer encoder constants
     * @return             This DrivetrainConstants object
     */
    public DrivetrainConstants withSteerEncoder(SteerEncoderConstants steerEncoder) {
        SteerEncoder = steerEncoder;
        return this;
    }

    /**
     * Sets the swerve module constants for the drivetrain, and returns this
     * DrivetrainConstants for chaining.
     * 
     * @param swerveModules The swerve module constants
     * @return              This DrivetrainConstants object
     */
    public DrivetrainConstants withSwerveModules(SwerveModuleConstants[] swerveModules) {
        SwerveModules = swerveModules;
        return this;
    }

    /**
     * Sets the driving constants for the drivetrain, and returns this
     * DrivetrainConstants for chaining.
     * 
     * @param driving The driving constants
     * @return        This DrivetrainConstants object
     */
    public DrivetrainConstants withDriving(DrivingConstants driving) {
        Driving = driving;
        return this;
    }

    /**
     * Gets the swerve module constants for the specified module index.
     * 
     * @param index The index of the swerve module
     * @return      The swerve module constants
     */
    public SwerveModuleConstants getSwerveModule(int index) {
        return SwerveModules[index];
    }

    /**
     * Gets the swerve module constants for the specified module corner.
     * 
     * @param corner The corner of the swerve drive
     * @return       The swerve module constants
     */
    public SwerveModuleConstants getSwerveModule(Corner corner) {
        return SwerveModules[corner.getIndex()];
    }

    /**
     * Gets the wheel radius of the specified swerve module. If the module has
     * a specific wheel radius configured, that radius is returned. Otherwise,
     * the default wheel radius from the StructureConstants is returned.
     * 
     * @param index The index of the swerve module
     * @return      The wheel radius of the swerve module
     */
    public Distance getWheelRadius(int index) {
        SwerveModuleConstants module = SwerveModules[index];
        if (module.WheelRadius != null) {
            return module.WheelRadius;
        } else {
            return Structure.WheelRadius;
        }
    }

    /**
     * Gets the wheel radius of the specified swerve module. If the module has
     * a specific wheel radius configured, that radius is returned. Otherwise,
     * the default wheel radius from the StructureConstants is returned.
     * 
     * @param corner The corner of the swerve drive
     * @return       The wheel radius of the swerve module
     */
    public Distance getWheelRadius(Corner corner) {
        return getWheelRadius(corner.getIndex());
    }
}
