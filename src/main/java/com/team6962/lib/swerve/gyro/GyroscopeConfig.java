package com.team6962.lib.swerve.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;

import edu.wpi.first.units.measure.Frequency;

public class GyroscopeConfig {
    public String canBusName;
    public Frequency updateFrequency;
    public int canID;
    public Pigeon2Configuration pigeonConfiguration;
}
