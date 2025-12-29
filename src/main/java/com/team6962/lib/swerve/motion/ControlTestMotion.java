package com.team6962.lib.swerve.motion;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.team6962.lib.swerve.MotionSwerveDrive;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;

import dev.doglog.DogLog;

public class ControlTestMotion implements SwerveMotion {
    private ControlRequest[] controlRequests;
    private MotionSwerveDrive swerveDrive;

    public ControlTestMotion(MotionSwerveDrive swerveDrive, ControlRequest[] controlRequests) {
        this.swerveDrive = swerveDrive;
        this.controlRequests = controlRequests;
    }

    public ControlTestMotion(MotionSwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        int moduleCount = swerveDrive.getModules().length;
        this.controlRequests = new ControlRequest[moduleCount * 2];
        for (int i = 0; i < moduleCount; i++) {
            this.controlRequests[2 * i] = new NeutralOut();
            this.controlRequests[2 * i + 1] = new NeutralOut();
        }
    }

    public synchronized void setDriveControl(Corner moduleCorner, ControlRequest driveRequest) {
        int moduleIndex = moduleCorner.getIndex();
        controlRequests[2 * moduleIndex] = driveRequest;
    }

    public synchronized void setSteerControl(Corner moduleCorner, ControlRequest steerRequest) {
        int moduleIndex = moduleCorner.getIndex();
        controlRequests[2 * moduleIndex + 1] = steerRequest;
    }

    public synchronized void setModuleControl(Corner moduleCorner, ControlRequest driveRequest, ControlRequest steerRequest) {
        int moduleIndex = moduleCorner.getIndex();
        controlRequests[2 * moduleIndex] = driveRequest;
        controlRequests[2 * moduleIndex + 1] = steerRequest;
    }

    public void setAllDriveControl(ControlRequest driveRequest) {
        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            setDriveControl(Corner.fromIndex(i), driveRequest);
        }
    }

    public void setAllSteerControl(ControlRequest steerRequest) {
        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            setSteerControl(Corner.fromIndex(i), steerRequest);
        }
    }

    public void setAllModuleControl(ControlRequest driveRequest, ControlRequest steerRequest) {
        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            setModuleControl(Corner.fromIndex(i), driveRequest, steerRequest);
        }
    }

    public void setFrontLeftDriveControl(ControlRequest driveRequest) {
        setDriveControl(Corner.FrontLeft, driveRequest);
    }

    public void setFrontRightDriveControl(ControlRequest driveRequest) {
        setDriveControl(Corner.FrontRight, driveRequest);
    }

    public void setBackLeftDriveControl(ControlRequest driveRequest) {
        setDriveControl(Corner.BackLeft, driveRequest);
    }

    public void setBackRightDriveControl(ControlRequest driveRequest) {
        setDriveControl(Corner.BackRight, driveRequest);
    }

    public void setFrontLeftSteerControl(ControlRequest steerRequest) {
        setSteerControl(Corner.FrontLeft, steerRequest);
    }

    public void setFrontRightSteerControl(ControlRequest steerRequest) {
        setSteerControl(Corner.FrontRight, steerRequest);
    }

    public void setBackLeftSteerControl(ControlRequest steerRequest) {
        setSteerControl(Corner.BackLeft, steerRequest);
    }

    public void setBackRightSteerControl(ControlRequest steerRequest) {
        setSteerControl(Corner.BackRight, steerRequest);
    }

    public ControlTestMotion withDriveControl(Corner moduleCorner, ControlRequest driveRequest) {
        setDriveControl(moduleCorner, driveRequest);
        return this;
    }

    public ControlTestMotion withSteerControl(Corner moduleCorner, ControlRequest steerRequest) {
        setSteerControl(moduleCorner, steerRequest);
        return this;
    }

    public ControlTestMotion withModuleControl(Corner moduleCorner, ControlRequest driveRequest, ControlRequest steerRequest) {
        setModuleControl(moduleCorner, driveRequest, steerRequest);
        return this;
    }

    public ControlTestMotion withAllDriveControl(ControlRequest driveRequest) {
        setAllDriveControl(driveRequest);
        return this;
    }

    public ControlTestMotion withAllSteerControl(ControlRequest steerRequest) {
        setAllSteerControl(steerRequest);
        return this;
    }

    public ControlTestMotion withAllModuleControl(ControlRequest driveRequest, ControlRequest steerRequest) {
        setAllModuleControl(driveRequest, steerRequest);
        return this;
    }

    public ControlTestMotion withFrontLeftDriveControl(ControlRequest driveRequest) {
        setFrontLeftDriveControl(driveRequest);
        return this;
    }

    public ControlTestMotion withFrontRightDriveControl(ControlRequest driveRequest) {
        setFrontRightDriveControl(driveRequest);
        return this;
    }

    public ControlTestMotion withBackLeftDriveControl(ControlRequest driveRequest) {
        setBackLeftDriveControl(driveRequest);
        return this;
    }

    public ControlTestMotion withBackRightDriveControl(ControlRequest driveRequest) {
        setBackRightDriveControl(driveRequest);
        return this;
    }

    public ControlTestMotion withFrontLeftSteerControl(ControlRequest steerRequest) {
        setFrontLeftSteerControl(steerRequest);
        return this;
    }

    public ControlTestMotion withFrontRightSteerControl(ControlRequest steerRequest) {
        setFrontRightSteerControl(steerRequest);
        return this;
    }

    public ControlTestMotion withBackLeftSteerControl(ControlRequest steerRequest) {
        setBackLeftSteerControl(steerRequest);
        return this;
    }

    public ControlTestMotion withBackRightSteerControl(ControlRequest steerRequest) {
        setBackRightSteerControl(steerRequest);
        return this;
    }

    @Override
    public synchronized void update(double deltaTimeSeconds) {
        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            swerveDrive.getModules()[i].setControl(controlRequests[2 * i], controlRequests[2 * i + 1]);
        }
    }

    @Override
    public synchronized void logTelemetry(String basePath) {
        if (!basePath.endsWith("/")) {
            basePath += "/";
        }
        
        DogLog.log(basePath + "Type", "ControlTest");
    }
}
