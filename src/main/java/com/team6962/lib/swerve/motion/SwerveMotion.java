package com.team6962.lib.swerve.motion;

public interface SwerveMotion extends AutoCloseable {
    public default void update(double deltaTimeSeconds) {
    }

    public default void logTelemetry(String basePath) {
    }

    public default SwerveMotion fuseWith(SwerveMotion other) {
        return null;
    }

    @Override
    public default void close() {
    }
}
