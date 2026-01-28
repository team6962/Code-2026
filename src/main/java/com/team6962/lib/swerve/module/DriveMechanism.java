package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.math.WheelMath;
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * Controls the drive motor of a swerve module, which rotates the wheel around its shaft.
 *
 * <p>This class contains a set of methods for getting information about the current state of the
 * drive motor, including its position, velocity, acceleration, applied voltage, and current draw.
 * Note that motion of the wheel is described in two ways: angular units refer to rotation of the
 * wheel about its shaft and linear units signify the linear movement of the wheel along the ground.
 *
 * <p>The {@link #setControl(ControlRequest)} can be used to apply control requests to the motor.
 *
 * <p>The {@link #close()} method releases the motor controller resources. This should be called
 * when the mechanism is no longer needed to ensure proper cleanup.
 *
 * @see SteerMechanism
 * @see SwerveModule
 */
public class DriveMechanism implements SwerveComponent, AutoCloseable {
  /** The corner of the robot that this drive mechanism is on. */
  private Corner corner;

  /** The drivetrain constants used to configure the mechanism. */
  private DrivetrainConstants constants;

  /** The TalonFX motor controller for the drive motor. */
  private TalonFX motor;

  // Status signals that allow us to get the current state of the mechanism
  private StatusSignal<Angle> positionSignal;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<AngularAcceleration> accelerationSignal;
  private StatusSignal<Voltage> appliedVoltageSignal;
  private StatusSignal<Current> statorCurrentSignal;
  private StatusSignal<Current> supplyCurrentSignal;

  // Cached state of the mechanism
  private Angle angularPosition = Radians.of(0);
  private AngularVelocity angularVelocity = RadiansPerSecond.of(0);
  private AngularAcceleration angularAcceleration = RadiansPerSecondPerSecond.of(0);
  private Distance linearPosition = Meters.of(0);
  private LinearVelocity linearVelocity = MetersPerSecond.of(0);
  private LinearAcceleration linearAcceleration = MetersPerSecondPerSecond.of(0);
  private Voltage appliedVoltage = Volts.of(0);
  private Current statorCurrent = Amps.of(0);
  private Current supplyCurrent = Amps.of(0);

  /** The last control request sent to the motor, for logging. */
  private ControlRequest lastControlRequest;

  /**
   * Creates a new DriveMechanism for the specified corner.
   *
   * <p>Initializes the TalonFX motor controller with the configuration from the drivetrain
   * constants and sets up all status signals for data acquisition.
   *
   * @param corner The corner of the robot this module is occupying
   * @param constants The drivetrain configuration constants
   */
  public DriveMechanism(Corner corner, DrivetrainConstants constants) {
    CANBus bus = new CANBus(constants.CANBusName);

    this.corner = corner;
    this.constants = constants;

    motor = new TalonFX(constants.getSwerveModule(corner.getIndex()).DriveMotorCANId, bus);
    constants.DriveMotor.DeviceConfiguration.Feedback.SensorToMechanismRatio =
        constants.DriveMotor.GearReduction;

    StatusUtil.check(motor.getConfigurator().apply(constants.DriveMotor.DeviceConfiguration));

    positionSignal = motor.getPosition(false);
    velocitySignal = motor.getVelocity(false);
    accelerationSignal = motor.getAcceleration(false);
    appliedVoltageSignal = motor.getMotorVoltage(false);
    statorCurrentSignal = motor.getStatorCurrent(false);
    supplyCurrentSignal = motor.getSupplyCurrent(false);
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
      supplyCurrentSignal
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
    return new ParentDevice[] {motor};
  }

  /**
   * Logs telemetry data about the drive mechanism. This method should never be called while
   * refreshing the status signals.
   *
   * @param basePath The base path that all data will be logged under
   */
  @Override
  public void logTelemetry(String basePath) {
    if (!basePath.endsWith("/")) {
      basePath += "/";
    }

    DogLog.log(basePath + "Position", getPosition().in(Meters));
    DogLog.log(basePath + "Velocity", getVelocity().in(MetersPerSecond));
    DogLog.log(basePath + "Acceleration", getAcceleration().in(MetersPerSecondPerSecond));
    DogLog.log(basePath + "AngularPosition", getAngularPosition().in(Radians));
    DogLog.log(basePath + "AngularVelocity", getAngularVelocity().in(RadiansPerSecond));
    DogLog.log(
        basePath + "AngularAcceleration", getAngularAcceleration().in(RadiansPerSecondPerSecond));
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
          basePath + "TargetAngularPosition",
          Rotations.of(positionControl.Position).in(Radians),
          Radians);
    } else if (DynamicPositionControlRequest.isDynamicPositionControlRequest(lastControlRequest)) {
      DynamicPositionControlRequest positionControl =
          new DynamicPositionControlRequest(lastControlRequest);

      DogLog.log(
          basePath + "TargetAngularPosition",
          Rotations.of(positionControl.Position).in(Radians),
          Radians);
    } else if (VelocityControlRequest.isVelocityControlRequest(lastControlRequest)) {
      VelocityControlRequest velocityControl = new VelocityControlRequest(lastControlRequest);

      DogLog.log(
          basePath + "TargetAngularVelocity",
          RotationsPerSecond.of(velocityControl.Velocity).in(RadiansPerSecond),
          RadiansPerSecond);
    }

    LoggingUtil.log(basePath + "ControlRequest", lastControlRequest);
  }

  /**
   * Updates the internal state of the drive mechanism. This method should never be called while
   * refreshing the status signals.
   *
   * @param deltaTimeSeconds The time elapsed since the last update, in seconds
   */
  @Override
  public synchronized void update(double deltaTimeSeconds) {
    if (constants.DriveMotor.PositionLatencyCompensation) {
      angularPosition =
          Rotations.of(
              BaseStatusSignal.getLatencyCompensatedValueAsDouble(positionSignal, velocitySignal));
    } else {
      angularPosition = positionSignal.getValue();
    }

    Distance wheelRadius = constants.getWheelRadius(corner);

    linearPosition = WheelMath.toLinear(angularPosition, wheelRadius);

    if (constants.DriveMotor.VelocityLatencyCompensation) {
      angularVelocity =
          RotationsPerSecond.of(
              BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                  velocitySignal, accelerationSignal));
    } else {
      angularVelocity = velocitySignal.getValue();
    }

    linearVelocity = WheelMath.toLinear(angularVelocity, wheelRadius);

    angularAcceleration = accelerationSignal.getValue();

    linearAcceleration = WheelMath.toLinear(angularAcceleration, wheelRadius);

    appliedVoltage = appliedVoltageSignal.getValue();
    statorCurrent = statorCurrentSignal.getValue();
    supplyCurrent = supplyCurrentSignal.getValue();
  }

  /**
   * Gets the angular position of the wheel (rotation around the shaft).
   *
   * @return The angular position
   */
  public synchronized Angle getAngularPosition() {
    return angularPosition;
  }

  /**
   * Gets the linear distance that the wheel has traveled along the ground.
   *
   * @return The linear position (distance traveled)
   */
  public synchronized Distance getPosition() {
    return linearPosition;
  }

  /**
   * Gets the angular velocity of the wheel.
   *
   * @return The angular velocity
   */
  public synchronized AngularVelocity getAngularVelocity() {
    return angularVelocity;
  }

  /**
   * Gets the linear velocity of the wheel along the ground.
   *
   * @return The linear velocity
   */
  public synchronized LinearVelocity getVelocity() {
    return linearVelocity;
  }

  /**
   * Gets the angular acceleration of the wheel.
   *
   * @return The angular acceleration
   */
  public synchronized AngularAcceleration getAngularAcceleration() {
    return angularAcceleration;
  }

  /**
   * Gets the linear acceleration of the wheel along the ground.
   *
   * @return The linear acceleration
   */
  public synchronized LinearAcceleration getAcceleration() {
    return linearAcceleration;
  }

  /**
   * Gets the voltage currently applied to the motor.
   *
   * @return The applied voltage
   */
  public synchronized Voltage getAppliedVoltage() {
    return appliedVoltage;
  }

  /**
   * Gets the stator current (torque-producing current) of the motor.
   *
   * @return The stator current
   */
  public synchronized Current getStatorCurrent() {
    return statorCurrent;
  }

  /**
   * Gets the supply current (battery drawn current) of the motor.
   *
   * @return The supply current
   */
  public synchronized Current getSupplyCurrent() {
    return supplyCurrent;
  }

  /**
   * Applies the given control request to the drive motor.
   *
   * @param controlRequest The {@link ControlRequest} to apply
   */
  public void setControl(ControlRequest controlRequest) {
    lastControlRequest = controlRequest;
    motor.setControl(controlRequest);
  }

  /** Closes the motor controller and releases resources. */
  @Override
  public void close() {
    motor.close();
  }
}
