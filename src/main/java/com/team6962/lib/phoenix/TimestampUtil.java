package com.team6962.lib.phoenix;

import edu.wpi.first.wpilibj.Timer;

public class TimestampUtil {
    public static double phoenixTimestampToFPGA(double timestamp) {
        return timestamp - System.currentTimeMillis() / 1000.0 + Timer.getFPGATimestamp();
    }
}
