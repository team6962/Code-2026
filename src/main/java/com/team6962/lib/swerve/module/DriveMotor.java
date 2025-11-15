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
import com.team6962.lib.phoenix.StatusUtil;
import com.team6962.lib.phoenix.TimestampUtil;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class DriveMotor {
    private SwerveModuleConfig config;

    private TalonFX motor;

    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<AngularAcceleration> accelerationSignal;

    private StatusSignal<Voltage> appliedVoltageSignal;
    private StatusSignal<Current> statorCurrentSignal;
    private StatusSignal<Current> supplyCurrentSignal;

    private ControlRequest lastControlRequest;

    public DriveMotor(SwerveModuleConfig config) {
        this.config = config;
        
        motor = new TalonFX(config.driveMotorID, config.canBusName);

        StatusUtil.check(motor.getConfigurator().apply(config.driveMotorConfiguration));

        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        accelerationSignal = motor.getAcceleration();

        appliedVoltageSignal = motor.getMotorVoltage();
        statorCurrentSignal = motor.getStatorCurrent();
        supplyCurrentSignal = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(config.updateFrequency, getStatusSignals());
    }

    public TalonFX getMotorController() {
        return motor;
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

    public Angle getAngularPosition() {
        return Rotations.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            positionSignal,
            velocitySignal
        ));
    }

    public Distance getPosition() {
        return Meters.of(
            config.wheelRadius.in(Meters) * getAngularPosition().in(Radians)
        );
    }

    public AngularVelocity getAngularVelocity() {
        return RotationsPerSecond.of(BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            velocitySignal, accelerationSignal
        ));
    }

    public LinearVelocity getVelocity() {
        return MetersPerSecond.of(
            config.wheelRadius.in(Meters) * getAngularVelocity().in(RadiansPerSecond)
        );
    }

    public AngularAcceleration getAngularAcceleration() {
        return accelerationSignal.getValue();
    }

    public LinearAcceleration getAcceleration() {
        return MetersPerSecondPerSecond.of(
            config.wheelRadius.in(Meters) * getAngularAcceleration().in(RadiansPerSecondPerSecond)
        );
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
