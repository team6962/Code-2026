package com.team6962.lib.swerve.config;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6962.lib.phoenix.control.ControlOutputType;
import com.team6962.lib.phoenix.control.PositionMotionProfileType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;

/** The constants that define the steer motor behavior. */
public class SteerMotorConstants implements Cloneable {
  /**
   * The TalonFX configuration for the steer motor. Some fields in this configuration may be
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

  /**
   * The gear reduction from the steer motor to the orientation of the wheel about the steer axis.
   */
  public double GearReduction = 1.0;

  /** The output units for control of the steer motor. */
  public ControlOutputType OutputType = ControlOutputType.VoltageFOC;

  /** Motion profile type for position control of the steer motor. */
  public PositionMotionProfileType PositionControlMotionProfile =
      PositionMotionProfileType.Exponential;

  /**
   * The slot index for position control of the steer motor. This should be the index of the slot in
   * the TalonFXConfiguration that has the position PID and feedforward constants configured.
   */
  public int PositionSlot = 0;

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
   * The moment of inertia of the steer mechanism for simulation. This includes the rotor inertia
   * plus any additional inertia from the gearing.
   */
  public MomentOfInertia SimulatedMomentOfInertia = KilogramSquareMeters.of(0.000174);

  /** Constructs a SteerMotorConstants object with default values. */
  public SteerMotorConstants() {}

  /**
   * Sets the device configuration for the steer motor, and returns this SteerMotorConstants for
   * chaining.
   *
   * @param deviceConfiguration The TalonFX configuration
   * @return This SteerMotorConstants object
   */
  public SteerMotorConstants withDeviceConfiguration(TalonFXConfiguration deviceConfiguration) {
    DeviceConfiguration = deviceConfiguration;
    return this;
  }

  /**
   * Sets the gear reduction for the steer motor, and returns this SteerMotorConstants for chaining.
   *
   * @param gearReduction The gear reduction
   * @return This SteerMotorConstants object
   */
  public SteerMotorConstants withGearReduction(double gearReduction) {
    GearReduction = gearReduction;
    return this;
  }

  /**
   * Sets the output type for control of the steer motor, and returns this SteerMotorConstants for
   * chaining.
   *
   * @param outputType The output type for control
   * @return This SteerMotorConstants object
   */
  public SteerMotorConstants withOutputType(ControlOutputType outputType) {
    OutputType = outputType;
    return this;
  }

  /**
   * Sets the motion profile type to use for position control of the steer motor, and returns this
   * SteerMotorConstants for chaining.
   *
   * @param profile The motion profile type for position control
   * @return This SteerMotorConstants object
   */
  public SteerMotorConstants withPositionControlMotionProfile(PositionMotionProfileType profile) {
    PositionControlMotionProfile = profile;
    return this;
  }

  /**
   * Sets the position slot index for the steer motor, and returns this SteerMotorConstants for
   * chaining.
   *
   * @param positionSlot The position slot index
   * @return This SteerMotorConstants object
   */
  public SteerMotorConstants withPositionSlot(int positionSlot) {
    PositionSlot = positionSlot;
    return this;
  }

  /**
   * Sets whether position latency compensation is enabled, and returns this SteerMotorConstants for
   * chaining.
   *
   * @param positionLatencyCompensation True if enabled
   * @return This SteerMotorConstants object
   */
  public SteerMotorConstants withPositionLatencyCompensation(boolean positionLatencyCompensation) {
    PositionLatencyCompensation = positionLatencyCompensation;
    return this;
  }

  /**
   * Sets whether velocity latency compensation is enabled, and returns this SteerMotorConstants for
   * chaining.
   *
   * @param velocityLatencyCompensation True if enabled
   * @return This SteerMotorConstants object
   */
  public SteerMotorConstants withVelocityLatencyCompensation(boolean velocityLatencyCompensation) {
    VelocityLatencyCompensation = velocityLatencyCompensation;
    return this;
  }

  /**
   * Sets the motor model to use for simulation, and returns this SteerMotorConstants for chaining.
   *
   * @param simulatedMotor The motor model for simulation
   * @return This SteerMotorConstants object
   */
  public SteerMotorConstants withSimulatedMotor(DCMotor simulatedMotor) {
    SimulatedMotor = simulatedMotor;
    return this;
  }

  /**
   * Sets the simulated moment of inertia for the steer mechanism, and returns this
   * SteerMotorConstants for chaining.
   *
   * @param simulatedMomentOfInertia The simulated moment of inertia
   * @return This SteerMotorConstants object
   */
  public SteerMotorConstants withSimulatedMomentOfInertia(
      MomentOfInertia simulatedMomentOfInertia) {
    SimulatedMomentOfInertia = simulatedMomentOfInertia;
    return this;
  }

  @Override
  public SteerMotorConstants clone() {
    try {
      return (SteerMotorConstants) super.clone();
    } catch (CloneNotSupportedException e) {
      throw new AssertionError("Clone should be supported", e);
    }
  }
}
