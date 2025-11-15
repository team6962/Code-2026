package com.team6962.lib.swerve.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;

public class SwerveModuleConfig {
    public String canBusName;
    public Frequency updateFrequency;

    public int driveMotorID;
    public TalonFXConfiguration driveMotorConfiguration;
    public Distance wheelRadius;

    public int steerMotorID;
    public TalonFXConfiguration steerMotorConfiguration;
    public int steerEncoderID;
    public CANcoderConfiguration steerEncoderConfiguration;
}
