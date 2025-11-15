package com.team6962.lib.swerve.module;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Current;

public class SwerveModule {
    private SwerveModuleConfig config;
    private DriveMotor driveMotor;
    private SteerMotor steerMotor;

    public SwerveModule(SwerveModuleConfig config) {
        this.config = config;
        driveMotor = new DriveMotor(config);
        steerMotor = new SteerMotor(config);
    }

    public SwerveModuleConfig getConfig() {
        return config;
    }

    public DriveMotor getDriveMotor() {
        return driveMotor;
    }

    public SteerMotor getSteerMotor() {
        return steerMotor;
    }

    public void logTelemetry(String basePath) {
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }

        driveMotor.logTelemetry(basePath + "Drive/");
        steerMotor.logTelemetry(basePath + "Steer/");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity(), new Rotation2d(steerMotor.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition(), new Rotation2d(steerMotor.getPosition()));
    }

    public Current getSupplyCurrent() {
        return driveMotor.getSupplyCurrent().plus(steerMotor.getSupplyCurrent());
    }

    public void setControl(ControlRequest driveRequest, ControlRequest steerRequest) {
        driveMotor.setControl(driveRequest);
        steerMotor.setControl(steerRequest);
    }
}
