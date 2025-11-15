package com.team6962.lib.swerve.simulation;

import com.team6962.lib.swerve.module.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleSim {
    private DriveMotorSim driveMotorSim;
    private SteerMotorSim steerMotorSim;

    public SwerveModuleSim(SimulationConfig simulationConfig, SwerveModule module) {
        driveMotorSim = new DriveMotorSim(module.getConfig(), simulationConfig, module.getDriveMotor().getMotorController());
        steerMotorSim = new SteerMotorSim(module.getConfig(), simulationConfig, module.getSteerMotor().getMotorController(), module.getSteerMotor().getEncoder());
    }

    public DriveMotorSim getDriveMotorSim() {
        return driveMotorSim;
    }

    public SteerMotorSim getSteerMotorSim() {
        return steerMotorSim;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotorSim.getPosition(),
            new Rotation2d(steerMotorSim.getAngularPosition())
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotorSim.getVelocity(),
            new Rotation2d(steerMotorSim.getAngularPosition())
        );
    }

    public void update(double deltaTimeSeconds) {
        driveMotorSim.update(deltaTimeSeconds);
        steerMotorSim.update(deltaTimeSeconds);
    }
}
