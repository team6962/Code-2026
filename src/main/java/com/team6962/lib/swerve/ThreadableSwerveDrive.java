package com.team6962.lib.swerve;

import com.ctre.phoenix6.controls.ControlRequest;
import com.team6962.lib.swerve.config.DrivetrainConstants;
import com.team6962.lib.swerve.config.SwerveModuleConstants.Corner;
import com.team6962.lib.swerve.core.SwerveControlLoop;
import com.team6962.lib.swerve.localization.Gyroscope;
import com.team6962.lib.swerve.localization.Localization;
import com.team6962.lib.swerve.localization.Odometry;
import com.team6962.lib.swerve.module.SwerveModule;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ThreadableSwerveDrive {
    private DrivetrainConstants constants;
    private SwerveControlLoop controlLoop;
    private SwerveModule[] modules;
    private Odometry odometry;
    private Gyroscope gyroscope;
    private Localization localization;

    public DrivetrainConstants getConstants() {
        return constants;
    }

    public SwerveModulePosition[] getModulePositions() {
        controlLoop.lockResources();

        try {
            return odometry.getPositions();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public SwerveModulePosition[] getModulePositionDeltas() {
        controlLoop.lockResources();

        try {
            return odometry.getPositionDeltas();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public SwerveModuleState[] getModuleStates() {
        controlLoop.lockResources();

        try {
            return odometry.getStates();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public Twist2d getTwist() {
        controlLoop.lockResources();

        try {
            return odometry.getTwist();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public Pose2d getPosition() {
        controlLoop.lockResources();
        
        try {
            return localization.getPosition();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public ChassisSpeeds getVelocity() {
        controlLoop.lockResources();

        try {
            return localization.getVelocity();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public Twist2d getArcVelocity() {
        controlLoop.lockResources();

        try {
            return localization.getArcVelocity();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public void addVisionMeasurement(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ) {
        controlLoop.lockResources();

        try {
            localization.addVisionEstimate(pose, timestampSeconds, stdDevs);
        } finally {
            controlLoop.unlockResources();
        }
    }

    public void addVisionEstimate(
        Pose2d pose,
        double timestampSeconds
    ) {
        controlLoop.lockResources();

        try {
            localization.addVisionEstimate(pose, timestampSeconds);
        } finally {
            controlLoop.unlockResources();
        }
    }

    public void setGyroscopeEnabled(boolean enabled) {
        controlLoop.lockResources();

        try {
            gyroscope.setEnabled(enabled);
        } finally {
            controlLoop.unlockResources();
        }
    }

    public Angle getHeading() {
        controlLoop.lockResources();

        try {
            return localization.getHeading();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public Angle getYaw() {
        return getHeading();
    }

    public AngularVelocity getYawVelocity() {
        controlLoop.lockResources();

        try {
            return gyroscope.getYawVelocity();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public Angle getPitch() {
        controlLoop.lockResources();

        try {
            return gyroscope.getPitch();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public AngularVelocity getPitchVelocity() {
        controlLoop.lockResources();

        try {
            return gyroscope.getPitchVelocity();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public Angle getRoll() {
        controlLoop.lockResources();

        try {
            return gyroscope.getRoll();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public AngularVelocity getRollVelocity() {
        controlLoop.lockResources();

        try {
            return gyroscope.getRollVelocity();
        } finally {
            controlLoop.unlockResources();
        }
    }

    public void setModuleControl(Corner corner, ControlRequest driveControlRequest, ControlRequest steerControlRequest) {
        controlLoop.lockResources();

        try {
            modules[corner.getIndex()].setControl(driveControlRequest, steerControlRequest);
        } finally {
            controlLoop.unlockResources();
        }
    }
}
