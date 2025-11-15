package com.team6962.lib.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6962.lib.logging.LoggingUtil;
import com.team6962.lib.phoenix.StatusUtil;
import com.team6962.lib.phoenix.TimestampUtil;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class SteerMotor {
    private TalonFX motor;
    private CANcoder encoder;

    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<AngularAcceleration> accelerationSignal;

    private StatusSignal<Voltage> appliedVoltageSignal;
    private StatusSignal<Current> statorCurrentSignal;
    private StatusSignal<Current> supplyCurrentSignal;

    private ControlRequest lastControlRequest;

    public SteerMotor(SwerveModuleConfig config) {
        motor = new TalonFX(config.steerMotorID, config.canBusName);

        StatusUtil.check(motor.getConfigurator().apply(config.steerMotorConfiguration));

        encoder = new CANcoder(config.steerEncoderID);

        encoder.getConfigurator().apply(config.steerEncoderConfiguration);

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        accelerationSignal = motor.getAcceleration();

        appliedVoltageSignal = motor.getMotorVoltage();
        statorCurrentSignal = motor.getStatorCurrent();
        supplyCurrentSignal = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(config.updateFrequency, getStatusSignals());
    }

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
        DogLog.log(basePath + "DataTimestamp", TimestampUtil.phoenixTimestampToFPGA(positionSignal.getTimestamp().getTime()));

        LoggingUtil.log(basePath + "ControlRequest", lastControlRequest);
    }

    public TalonFX getMotorController() {
        return motor;
    }

    public CANcoder getEncoder() {
        return encoder;
    }

    public Angle getPosition() {
        return Rotations.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            positionSignal,
            velocitySignal
        ));
    }

    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            velocitySignal, accelerationSignal
        ));
    }
    public AngularAcceleration getAcceleration() {
        return accelerationSignal.getValue();
    }

    public Voltage getAppliedVoltage() {
        return appliedVoltageSignal.getValue();
    }

    public Current getStatorCurrent() {
        return statorCurrentSignal.getValue();
    }

    public Current getSupplyCurrent() {
        return supplyCurrentSignal.getValue();
    }

    public void setControl(ControlRequest controlRequest) {
        lastControlRequest = controlRequest;
        motor.setControl(controlRequest);
    }
}
