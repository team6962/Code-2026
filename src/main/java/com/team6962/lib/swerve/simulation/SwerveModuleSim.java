package com.team6962.lib.swerve.simulation;

import com.team6962.lib.swerve.module.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleSim {
    private DriveMechanismSim driveMechanismSim;
    private SteerMechanismSim steerMechanismSim;

    public SwerveModuleSim(SwerveModule module) {
        driveMechanismSim = new DriveMechanismSim(module.getCorner(), module.getConstants(), module.getDriveMechanism().getMotorController());
        steerMechanismSim = new SteerMechanismSim(module.getCorner(), module.getConstants(), module.getSteerMechanism().getMotorController(), module.getSteerMechanism().getEncoder());
    }

    public DriveMechanismSim getDriveMechanism() {
        return driveMechanismSim;
    }

    public SteerMechanismSim getSteerMechanism() {
        return steerMechanismSim;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMechanismSim.getPosition(),
            new Rotation2d(steerMechanismSim.getAngularPosition())
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMechanismSim.getVelocity(),
            new Rotation2d(steerMechanismSim.getAngularPosition())
        );
    }

    public void update(double deltaTimeSeconds) {
        driveMechanismSim.update(deltaTimeSeconds);
        steerMechanismSim.update(deltaTimeSeconds);
    }
}
