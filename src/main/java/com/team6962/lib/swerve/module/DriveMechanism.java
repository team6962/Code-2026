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
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.math.WheelMath;
import com.team6962.lib.phoenix.StatusUtil;
import com.team6962.lib.phoenix.TimestampUtil;
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

public class DriveMechanism implements SwerveComponent, AutoCloseable {
    private Corner corner;
    private DrivetrainConstants constants;

    private TalonFX motor;

    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<AngularAcceleration> accelerationSignal;
    private StatusSignal<Voltage> appliedVoltageSignal;
    private StatusSignal<Current> statorCurrentSignal;
    private StatusSignal<Current> supplyCurrentSignal;

    private Angle angularPosition = Radians.of(0);
    private AngularVelocity angularVelocity = RadiansPerSecond.of(0);
    private AngularAcceleration angularAcceleration = RadiansPerSecondPerSecond.of(0);
    private Distance linearPosition = Meters.of(0);
    private LinearVelocity linearVelocity = MetersPerSecond.of(0);
    private LinearAcceleration linearAcceleration = MetersPerSecondPerSecond.of(0);
    private Voltage appliedVoltage = Volts.of(0);
    private Current statorCurrent = Amps.of(0);
    private Current supplyCurrent = Amps.of(0);

    private ControlRequest lastControlRequest;

    public DriveMechanism(Corner corner, DrivetrainConstants constants) {
        this.corner = corner;
        this.constants = constants;
        
        motor = new TalonFX(constants.getSwerveModule(corner.getIndex()).DriveMotorCANId, constants.CANBusName);

        constants.DriveMotor.DeviceConfiguration.Feedback.SensorToMechanismRatio = constants.DriveMotor.GearReduction;

        StatusUtil.check(motor.getConfigurator().apply(constants.DriveMotor.DeviceConfiguration));

        positionSignal = motor.getPosition(false);
        velocitySignal = motor.getVelocity(false);
        accelerationSignal = motor.getAcceleration(false);
        appliedVoltageSignal = motor.getMotorVoltage(false);
        statorCurrentSignal = motor.getStatorCurrent(false);
        supplyCurrentSignal = motor.getSupplyCurrent(false);
    }

    public TalonFX getMotorController() {
        return motor;
    }

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
     * Logs telemetry data about the drive mechanism. This method should never
     * be called while refreshing the status signals.
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
        DogLog.log(basePath + "AngularAcceleration", getAngularAcceleration().in(RadiansPerSecondPerSecond));
        DogLog.log(basePath + "AppliedVoltage", getAppliedVoltage().in(Volts));
        DogLog.log(basePath + "StatorCurrent", getStatorCurrent().in(Amps));
        DogLog.log(basePath + "SupplyCurrent", getSupplyCurrent().in(Amps));
        DogLog.log(basePath + "DataTimestamp", TimestampUtil.phoenixTimestampToFPGA(positionSignal.getTimestamp().getTime()));

        LoggingUtil.log(basePath + "ControlRequest", lastControlRequest);
    }

    /**
     * Updates the internal state of the drive mechanism. This method should
     * never be called while refreshing the status signals.
     */
    @Override
    public synchronized void update(double deltaTimeSeconds) {
        angularPosition = Rotations.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            positionSignal, velocitySignal
        ));

        Distance wheelRadius = constants.getWheelRadius(corner);

        linearPosition = WheelMath.toLinear(angularPosition, wheelRadius);

        angularVelocity = RotationsPerSecond.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            velocitySignal, accelerationSignal
        ));

        linearVelocity = WheelMath.toLinear(angularVelocity, wheelRadius);

        angularAcceleration = accelerationSignal.getValue();

        linearAcceleration = WheelMath.toLinear(angularAcceleration, wheelRadius);

        appliedVoltage = appliedVoltageSignal.getValue();
        statorCurrent = statorCurrentSignal.getValue();
        supplyCurrent = supplyCurrentSignal.getValue();
    }

    public synchronized Angle getAngularPosition() {
        return angularPosition;
    }

    public synchronized Distance getPosition() {
        return linearPosition;
    }

    public synchronized AngularVelocity getAngularVelocity() {
        return angularVelocity;
    }

    public synchronized LinearVelocity getVelocity() {
        return linearVelocity;
    }

    public synchronized AngularAcceleration getAngularAcceleration() {
        return angularAcceleration;
    }

    public synchronized LinearAcceleration getAcceleration() {
        return linearAcceleration;
    }

    public synchronized Voltage getAppliedVoltage() {
        return appliedVoltage;
    }

    public synchronized Current getStatorCurrent() {
        return statorCurrent;
    }

    public synchronized Current getSupplyCurrent() {
        return supplyCurrent;
    }

    public void setControl(ControlRequest controlRequest) {
        lastControlRequest = controlRequest;
        motor.setControl(controlRequest);
    }

    @Override
    public void close() {
        motor.close();
    }
}
