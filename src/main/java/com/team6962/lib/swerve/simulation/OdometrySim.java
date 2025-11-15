package com.team6962.lib.swerve.simulation;

import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;

public class OdometrySim {
    private SwerveModuleSim[] moduleSims;
    private SwerveDriveKinematics kinematics;
    private SwerveModulePosition[] modulePositions;
    private Angle deltaYaw;

    public OdometrySim(SwerveDriveKinematics kinematics, SwerveModuleSim[] moduleSims) {
        this.moduleSims = moduleSims;
        this.kinematics = kinematics;
        this.deltaYaw = Radians.of(0);
        this.modulePositions = new SwerveModulePosition[4];
        refreshModulePositions();
    }

    private void refreshModulePositions() {
        for (int i = 0; i < moduleSims.length; i++) {
            modulePositions[i] = moduleSims[i].getPosition();
        }
    }

    public Angle getDeltaYaw() {
        return deltaYaw;
    }

    public void update() {
        SwerveModulePosition[] previousPositions = Arrays.copyOf(modulePositions, modulePositions.length);

        refreshModulePositions();

        Twist2d twist = kinematics.toTwist2d(previousPositions, modulePositions);

        deltaYaw = Radians.of(twist.dtheta);
    }
}
