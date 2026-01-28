package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.phoenix.StatusUtil;
import com.team6962.lib.phoenix.control.DynamicPositionControlRequest;
import com.team6962.lib.phoenix.control.PositionControlRequest;
import com.team6962.lib.phoenix.control.VelocityControlRequest;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;
import com.team6962.lib.swerve.util.SwerveComponent;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Controls the steer motor of a swerve module, which rotates the wheel around its vertical axis.
 *
 * <p>This class contains a set of methods for getting information about the current state of the
 * steer motor, including its position, velocity, acceleration, applied voltage, and current draw.
 * All measurements are in angular units representing the rotation of the wheel about its vertical
 * axis.
 *
 * <p>The {@link #setControl(ControlRequest)} can be used to apply control requests to the motor.
 *
 * <p>The {@link #close()} method releases the motor controller resources. This should be called
 * when the mechanism is no longer needed to ensure proper cleanup.
 *
 * @see DriveMechanism
 * @see SwerveModule
 */
public class SteerMechanism implements SwerveComponent, AutoCloseable {
  /** The drivetrain constants used to configure the mechanism. */
  private DrivetrainConstants constants;

  /** The TalonFX motor controller for the steer motor. */
  private TalonFX motor;

  /** The CANcoder absolute encoder for that reads the steer angle. */
  private CANcoder encoder;

  // Status signals that allow us to get the current state of the mechanism
  private StatusSignal<Angle> positionSignal;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<AngularAcceleration> accelerationSignal;
  private StatusSignal<Voltage> appliedVoltageSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Current> supplyCurrentSignal;

  // Cached values for the current state of the mechanism
  private Angle position = Radians.of(0);
  private AngularVelocity velocity = RadiansPerSecond.of(0);
  private AngularAcceleration acceleration = RadiansPerSecondPerSecond.of(0);
  private Voltage appliedVoltage = Volts.of(0);
  private Current statorCurrent = Amps.of(0);
  private Current supplyCurrent = Amps.of(0);

  /** The last control request sent to the motor for telemetry logging. */
  private ControlRequest lastControlRequest;

  /**
   * Creates a new SteerMechanism for the specified corner.
   *
   * <p>Initializes the TalonFX motor controller and CANcoder with the configuration from the
   * drivetrain constants. Configures the motor to use FusedCANcoder feedback for accurate absolute
   * angle tracking.
   *
   * @param corner The corner of the robot that this module is occupying
   * @param constants The drivetrain configuration constants
   */
  public SteerMechanism(Corner corner, DrivetrainConstants constants) {
    this.constants = constants;

    CANBus bus = new CANBus(constants.CANBusName);

    motor = new TalonFX(constants.getSwerveModule(corner).SteerMotorCANId, bus);
    encoder = new CANcoder(constants.getSwerveModule(corner).SteerEncoderCANId, bus);

    constants.SteerMotor.DeviceConfiguration.Feedback.RotorToSensorRatio =
        constants.SteerMotor.GearReduction;
    constants.SteerMotor.DeviceConfiguration.Feedback.SensorToMechanismRatio = 1.0;
    constants.SteerMotor.DeviceConfiguration.Feedback.FeedbackSensorSource =
        constants.SteerEncoder.DataFusion.feedbackSensorSource;
    constants.SteerMotor.DeviceConfiguration.Feedback.FeedbackRemoteSensorID =
        constants.getSwerveModule(corner).SteerEncoderCANId;

    CANcoderConfiguration encoderConfig = constants.SteerEncoder.DeviceConfiguration.clone();

    // Add an offset for each module that accounts for each module's
    // rotation on the robot, to make module swapping easier
    encoderConfig.MagnetSensor.MagnetOffset =
        constants
            .getSwerveModule(corner)
            .SteerEncoderOffset
            .minus(corner.getRotation())
            .in(Rotations);

    StatusUtil.check(motor.getConfigurator().apply(constants.SteerMotor.DeviceConfiguration));
    StatusUtil.check(encoder.getConfigurator().apply(encoderConfig));

    positionSignal = motor.getPosition(false);
    velocitySignal = motor.getVelocity(false);
    accelerationSignal = motor.getAcceleration(false);
    appliedVoltageSignal = motor.getMotorVoltage(false);
    statorCurrentSignal = motor.getStatorCurrent(false);
    supplyCurrentSignal = motor.getSupplyCurrent(false);
  }

  /**
   * Gets the status signals used by this mechanism. These signals should be refreshed periodically
   * by the containing class.
   *
   * @return Array of status signals for position, velocity, acceleration, voltage, and currents
   */
  @Override
  public BaseStatusSignal[] getStatusSignals() {
    return new BaseStatusSignal[] {
      positionSignal,
      velocitySignal,
      accelerationSignal,
      appliedVoltageSignal,
      statorCurrentSignal,
      supplyCurrentSignal,
      // required for encoder data fusion to work
      encoder.getPosition(),
      encoder.getVelocity()
    };
  }

  /**
   * Gets the list of Phoenix devices connected to by this component. All devices in this list will
   * have their bus utilization optimized in parallel.
   *
   * @return an array of Phoenix devices used by this component
   */
  @Override
  public ParentDevice[] getPhoenixDevices() {
    return new ParentDevice[] {motor, encoder};
  }

  /**
   * Logs telemetry data about the steer mechanism. This method should never be called while
   * refreshing the status signals.
   */
  @Override
  public void logTelemetry(String basePath) {
    if (!basePath.endsWith("/")) {
      basePath += "/";
    }

    DogLog.log(basePath + "Position", getPosition().in(Radians));
    DogLog.log(basePath + "Velocity", getVelocity().in(RadiansPerSecond));
    DogLog.log(basePath + "Acceleration", getAcceleration().in(RadiansPerSecondPerSecond));
    DogLog.log(basePath + "AppliedVoltage", getAppliedVoltage().in(Volts));
    DogLog.log(basePath + "StatorCurrent", getStatorCurrent().in(Amps));
    DogLog.log(basePath + "SupplyCurrent", getSupplyCurrent().in(Amps));
    DogLog.log(
        basePath + "DataTimestamp",
        StatusUtil.toFPGATimestamp(positionSignal.getTimestamp().getTime()));

    // If running a position or velocity request, log the target position or
    // velocity in radians. The target is logged as part of the control
    // request in rotations already, but it is more convenient if the target
    // is logged in radians as well.
    if (PositionControlRequest.isPositionControlRequest(lastControlRequest)) {
      PositionControlRequest positionControl = new PositionControlRequest(lastControlRequest);

      DogLog.log(
          basePath + "TargetPosition", Rotations.of(positionControl.Position).in(Radians), Radians);
    } else if (DynamicPositionControlRequest.isDynamicPositionControlRequest(lastControlRequest)) {
      DynamicPositionControlRequest positionControl =
          new DynamicPositionControlRequest(lastControlRequest);

      DogLog.log(
          basePath + "TargetPosition", Rotations.of(positionControl.Position).in(Radians), Radians);
    } else if (VelocityControlRequest.isVelocityControlRequest(lastControlRequest)) {
      VelocityControlRequest velocityControl = new VelocityControlRequest(lastControlRequest);

      DogLog.log(
          basePath + "TargetVelocity",
          RotationsPerSecond.of(velocityControl.Velocity).in(RadiansPerSecond),
          RadiansPerSecond);
    }

    LoggingUtil.log(basePath + "ControlRequest", lastControlRequest);
  }

  /**
   * Updates the internal state of the steer mechanism. This method should never be called while
   * refreshing the status signals.
   */
  @Override
  public synchronized void update(double deltaTimeSeconds) {
    if (constants.SteerMotor.PositionLatencyCompensation) {
      position =
          Rotations.of(
              BaseStatusSignal.getLatencyCompensatedValueAsDouble(positionSignal, velocitySignal));
    } else {
      position = positionSignal.getValue();
    }

    if (constants.SteerMotor.VelocityLatencyCompensation) {
      velocity =
          RotationsPerSecond.of(
              BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                  velocitySignal, accelerationSignal));
    } else {
      velocity = velocitySignal.getValue();
    }

    acceleration = accelerationSignal.getValue();

    appliedVoltage = appliedVoltageSignal.getValue();
    statorCurrent = statorCurrentSignal.getValue();
    supplyCurrent = supplyCurrentSignal.getValue();
  }

  /**
   * Gets the TalonFX motor controller for direct access.
   *
   * @return The TalonFX motor controller
   */
  public TalonFX getMotorController() {
    return motor;
  }

  /**
   * Gets the CANcoder absolute encoder for direct access.
   *
   * @return The CANcoder encoder
   */
  public CANcoder getEncoder() {
    return encoder;
  }

  /**
   * Gets the current angular position of the wheel about the steer axis.
   *
   * <p>This is the fused position from the TalonFX and CANcoder, providing accurate absolute angle
   * tracking.
   *
   * @return The angular position of the wheel
   */
  public synchronized Angle getPosition() {
    return position;
  }

  /**
   * Gets the angular velocity of the wheel about the steer axis.
   *
   * @return The angular velocity
   */
  public synchronized AngularVelocity getVelocity() {
    return velocity;
  }

  /**
   * Gets the angular acceleration of the wheel about the steer axis.
   *
   * @return The angular acceleration
   */
  public synchronized AngularAcceleration getAcceleration() {
    return acceleration;
  }

  /**
   * Gets the voltage currently applied to the steer motor.
   *
   * @return The applied voltage
   */
  public synchronized Voltage getAppliedVoltage() {
    return appliedVoltage;
  }

  /**
   * Gets the stator current (torque-producing current) of the steer motor.
   *
   * @return The stator current
   */
  public synchronized Current getStatorCurrent() {
    return statorCurrent;
  }

  /**
   * Gets the supply current (battery drawn current) of the steer motor.
   *
   * @return The supply current
   */
  public synchronized Current getSupplyCurrent() {
    return supplyCurrent;
  }

  /**
   * Applies the given control request to the steer motor.
   *
   * @param controlRequest The {@link ControlRequest} to apply
   */
  public void setControl(ControlRequest controlRequest) {
    lastControlRequest = controlRequest;
    motor.setControl(controlRequest);
  }

  /** Closes the motor controller and encoder, releasing resources. */
  @Override
  public void close() {
    motor.close();
    encoder.close();
  }
}
