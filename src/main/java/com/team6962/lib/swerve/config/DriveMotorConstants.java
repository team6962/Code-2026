package com.team6962.lib.swerve.config;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6962.lib.phoenix.control.ControlOutputType;
import com.team6962.lib.phoenix.control.DynamicPositionMotionProfileType;
import com.team6962.lib.phoenix.control.PositionMotionProfileType;
import com.team6962.lib.phoenix.control.VelocityMotionProfileType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

/** The constants that define the drive motor behavior. */
public class DriveMotorConstants implements Cloneable {
  /**
   * The TalonFX configuration for the drive motor. Some fields in this configuration may be
   * overriden by other settings, such as the gear reduction.
   *
   * <p>Some settings thay you may want to configure:
   *
   * <ul>
   *   <li>Inversion
   *   <li>Neutral Mode
   *   <li>Current Limits
   *   <li>Motion Magic
   *   <li>Slot Configs
   * </ul>
   */
  public TalonFXConfiguration DeviceConfiguration = new TalonFXConfiguration();

  /** The gear reduction from the drive motor to the wheel. */
  public double GearReduction = 1.0;

  /** The output units for control of the drive motor. */
  public ControlOutputType OutputType = ControlOutputType.VoltageFOC;

  /** Motion profile type for position control of the drive motor. */
  public PositionMotionProfileType PositionControlMotionProfile =
      PositionMotionProfileType.Trapezoidal;

  /**
   * The slot index for position control of the drive motor. This should be the index of the slot in
   * the TalonFXConfiguration that has the position PID and feedforward constants configured.
   */
  public int PositionSlot = 0;

  /** Motion profile type for velocity control of the drive motor. */
  public VelocityMotionProfileType VelocityControlMotionProfile =
      VelocityMotionProfileType.Trapezoidal;

  /**
   * The slot index for velocity control of the drive motor. This should be the index of the slot in
   * the TalonFXConfiguration that has the velocity PID and feedforward constants configured.
   */
  public int VelocitySlot = 1;

  /** Motion profile type for dynamically-constrained position control of the drive motor. */
  public DynamicPositionMotionProfileType DynamicPositionControlMotionProfile =
      DynamicPositionMotionProfileType.Trapezoidal;

  /**
   * True if latency compensation should be performed on position data from the motor's internal
   * encoder. The latency compensation uses the velocity data to extrapolate the position to account
   * for the time delay between when the position was measured and when the control loop runs.
   */
  public boolean PositionLatencyCompensation = true;

  /**
   * True if latency compensation should be performed on velocity data from the motor's internal
   * encoder. The latency compensation uses the acceleration to extrapolate the velocity to account
   * for the time delay between when the velocity was measured and when the control loop runs.
   */
  public boolean VelocityLatencyCompensation = true;

  /**
   * The motor model to use for simulation. This should match the physical motor used on the robot.
   */
  public DCMotor SimulatedMotor = DCMotor.getKrakenX60Foc(1);

  /**
   * The moment of inertia of the drive mechanism for simulation. This includes the rotor inertia
   * plus any additional inertia from the wheel and gearing.
   */
  public MomentOfInertia SimulatedMomentOfInertia = KilogramSquareMeters.of(0.00032);

  /**
   * The maximum linear velocity of the drive wheel. This is used for wheel velocity desaturation.
   */
  public LinearVelocity MaxVelocity;

  /** Constructs a DriveMotorConstants object with default values. */
  public DriveMotorConstants() {}

  /**
   * Sets the device configuration for the drive motor, and returns this DriveMotorConstants for
   * chaining.
   *
   * @param deviceConfiguration The TalonFX configuration
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withDeviceConfiguration(TalonFXConfiguration deviceConfiguration) {
    DeviceConfiguration = deviceConfiguration;
    return this;
  }

  /**
   * Sets the gear reduction for the drive motor, and returns this DriveMotorConstants for chaining.
   *
   * @param gearReduction The gear reduction
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withGearReduction(double gearReduction) {
    GearReduction = gearReduction;
    return this;
  }

  /**
   * Sets the output type for control of the drive motor, and returns this DriveMotorConstants for
   * chaining.
   *
   * @param outputType The output type for control
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withOutputType(ControlOutputType outputType) {
    OutputType = outputType;
    return this;
  }

  /**
   * Sets the motion profile type to use for position control of the drive motor, and returns this
   * DriveMotorConstants for chaining.
   *
   * @param profile The motion profile type for position control
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withPositionControlMotionProfile(PositionMotionProfileType profile) {
    PositionControlMotionProfile = profile;
    return this;
  }

  /**
   * Sets the position slot index for the drive motor, and returns this DriveMotorConstants for
   * chaining.
   *
   * @param positionSlot The position slot index
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withPositionSlot(int positionSlot) {
    PositionSlot = positionSlot;
    return this;
  }

  /**
   * Sets the motion profile type to use for velocity control of the drive motor, and returns this
   * DriveMotorConstants for chaining.
   *
   * @param profile The motion profile type for velocity control
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withVelocityControlMotionProfile(VelocityMotionProfileType profile) {
    VelocityControlMotionProfile = profile;
    return this;
  }

  /**
   * Sets the velocity slot index for the drive motor, and returns this DriveMotorConstants for
   * chaining.
   *
   * @param velocitySlot The velocity slot index
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withVelocitySlot(int velocitySlot) {
    VelocitySlot = velocitySlot;
    return this;
  }

  /**
   * Sets the motion profile type to use for dynamically-constrained position control of the drive
   * motor, and returns this DriveMotorConstants for chaining.
   *
   * @param profile The motion profile type for dynamically-constrained position control
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withDynamicPositionControlMotionProfile(
      DynamicPositionMotionProfileType profile) {
    DynamicPositionControlMotionProfile = profile;
    return this;
  }

  /**
   * Sets whether position latency compensation is enabled, and returns this DriveMotorConstants for
   * chaining.
   *
   * @param positionLatencyCompensation True if enabled
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withPositionLatencyCompensation(boolean positionLatencyCompensation) {
    PositionLatencyCompensation = positionLatencyCompensation;
    return this;
  }

  /**
   * Sets whether velocity latency compensation is enabled, and returns this DriveMotorConstants for
   * chaining.
   *
   * @param velocityLatencyCompensation True if enabled
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withVelocityLatencyCompensation(boolean velocityLatencyCompensation) {
    VelocityLatencyCompensation = velocityLatencyCompensation;
    return this;
  }

  /**
   * Sets the motor model to use for simulation, and returns this DriveMotorConstants for chaining.
   *
   * @param simulatedMotor The motor model for simulation
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withSimulatedMotor(DCMotor simulatedMotor) {
    SimulatedMotor = simulatedMotor;
    return this;
  }

  /**
   * Sets the moment of inertia of the drive mechanism for simulation, and returns this
   * DriveMotorConstants for chaining.
   *
   * @param simulatedMomentOfInertia The moment of inertia for simulation
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withSimulatedMomentOfInertia(
      MomentOfInertia simulatedMomentOfInertia) {
    SimulatedMomentOfInertia = simulatedMomentOfInertia;
    return this;
  }

  /**
   * Sets the maximum linear velocity of the drive wheel, and returns this DriveMotorConstants for
   * chaining.
   *
   * @param maxVelocity The maximum linear velocity
   * @return This DriveMotorConstants object
   */
  public DriveMotorConstants withMaxVelocity(LinearVelocity maxVelocity) {
    MaxVelocity = maxVelocity;
    return this;
  }

  @Override
  public DriveMotorConstants clone() {
    try {
      return (DriveMotorConstants) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone should be supported", e);
    }
  }
}
