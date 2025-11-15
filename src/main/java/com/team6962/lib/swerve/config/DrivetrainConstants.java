package com.team6962.lib.swerve.config;

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

    
}
