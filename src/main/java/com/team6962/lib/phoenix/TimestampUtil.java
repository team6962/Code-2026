package com.team6962.lib.phoenix;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Timer;

public class TimestampUtil {
    public static double phoenixTimestampToFPGA(double timestamp) {
        return timestamp + Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds();
    }
}
