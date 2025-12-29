package com.team6962.lib.swerve.motion;

public interface SwerveMotion extends AutoCloseable {
    public default void update(double deltaTimeSeconds) {
    }

    public default void logTelemetry(String basePath) {
    }

    public default SwerveMotion fuseWith(SwerveMotion other) {
        throw new IllegalArgumentException("Cannot fuse a " + getClass().getSimpleName() + " with a " + other.getClass().getSimpleName());
    }

    @Override
    public default void close() {
    }
}
