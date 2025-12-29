package com.team6962.lib.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;
import com.team6962.lib.swerve.util.SwerveComponent;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Current;

public class SwerveModule implements SwerveComponent, AutoCloseable {
    private Corner corner;
    private DrivetrainConstants constants;
    private DriveMechanism driveMechanism;
    private SteerMechanism steerMechanism;

    public SwerveModule(Corner corner, DrivetrainConstants constants) {
        this.corner = corner;
        this.constants = constants;

        driveMechanism = new DriveMechanism(corner, constants);
        steerMechanism = new SteerMechanism(corner, constants);
    }

    @Override
    public BaseStatusSignal[] getStatusSignals() {
        return SwerveComponent.combineStatusSignals(driveMechanism, steerMechanism);
    }

    @Override
    public void update(double deltaTimeSeconds) {
        driveMechanism.update(deltaTimeSeconds);
        steerMechanism.update(deltaTimeSeconds);
    }

    public Corner getCorner() {
        return corner;
    }

    public int getIndex() {
        return corner.getIndex();
    }

    public DrivetrainConstants getConstants() {
        return constants;
    }

    public DriveMechanism getDriveMechanism() {
        return driveMechanism;
    }

    public SteerMechanism getSteerMechanism() {
        return steerMechanism;
    }

    @Override
    public void logTelemetry(String basePath) {
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }

        driveMechanism.logTelemetry(basePath + "Drive/");
        steerMechanism.logTelemetry(basePath + "Steer/");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMechanism.getVelocity(), new Rotation2d(steerMechanism.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMechanism.getPosition(), new Rotation2d(steerMechanism.getPosition()));
    }

    public Current getSupplyCurrent() {
        return driveMechanism.getSupplyCurrent().plus(steerMechanism.getSupplyCurrent());
    }

    public void setControl(ControlRequest driveRequest, ControlRequest steerRequest) {
        driveMechanism.setControl(driveRequest);
        steerMechanism.setControl(steerRequest);
    }

    @Override
    public void close() {
        driveMechanism.close();
        steerMechanism.close();
    }
}
