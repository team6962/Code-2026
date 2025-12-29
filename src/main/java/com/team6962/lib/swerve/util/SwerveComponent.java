package com.team6962.lib.swerve.util;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix6.BaseStatusSignal;

public interface SwerveComponent {
    /**
     * Gets the status signals used by this component for monitoring and
     * diagnostics. These signals will be periodically refreshed by the
     * containing class.
     * 
     * @return An array of status signals used by this component.
     */
    public default BaseStatusSignal[] getStatusSignals() {
        return new BaseStatusSignal[0];
    }

    /**
     * Logs telemetry data for this component.
     * 
     * @param basePath A base path in the telemetry hierarchy which this
     *                 component's logged paths should be confined to.
     */
    public default void logTelemetry(String basePath) {
    }

    /**
     * Updates the component's internal state. This method should be called
     * at the configured control update frequency.
     * 
     * @param deltaTimeSeconds The time elapsed since the last update, in seconds.
     */
    public default void update(double deltaTimeSeconds) {
    }

    /**
     * Combines the status signals from multiple SwerveComponents into a single
     * array, removing duplicates.
     * 
     * @param components The SwerveComponents to combine status signals from.
     * @return An array of unique status signals from all provided components.
     */
    public static BaseStatusSignal[] combineStatusSignals(SwerveComponent... components) {
        Set<BaseStatusSignal> signalSet = new HashSet<>();

        for (SwerveComponent component : components) {
            BaseStatusSignal[] signals = component.getStatusSignals();
            for (BaseStatusSignal signal : signals) {
                signalSet.add(signal);
            }
        }

        return signalSet.toArray(new BaseStatusSignal[0]);
    }
}
