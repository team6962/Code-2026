package com.team6962.lib.swerve.motion;

/**
 * Interface representing a motion that a swerve drive can perform during a single control loop iteration.
 * 
 * <p>A SwerveMotion encapsulates the logic for controlling swerve module states to achieve
 * a specific type of movement. Each iteration, the {@link #update(double)} method is called
 * to apply the motion's control outputs to the swerve modules.
 * 
 * <p>Implementations of this interface include:
 * <ul>
 *   <li>{@link VelocityMotion} - Drives at a specified field-relative velocity</li>
 *   <li>{@link LockMotion} - Locks wheels in an X pattern to resist pushing</li>
 *   <li>{@link NeutralMotion} - Sets motors to neutral (coast or brake)</li>
 *   <li>{@link TwistToTargetMotion} - Drives along a twist path to a target pose</li>
 *   <li>{@link ControlTestMotion} - Drives along a twist path to a target pose</li>
 * </ul>
 * 
 * <p>Motions can be fused together using {@link #fuseWith(SwerveMotion)} to combine
 * compatible motion types (e.g., combining translation-only and rotation-only velocity motions).
 * 
 * <p>This interface extends {@link AutoCloseable} to allow cleanup of resources when
 * a motion is replaced or the swerve drive is closed.
 * 
 * @see SwerveMotionManager
 */
public interface SwerveMotion extends AutoCloseable {
    /**
     * Updates the motion and applies control outputs to the swerve modules.
     * 
     * <p>This method is called once per control loop iteration to compute and
     * apply the motor control requests needed to execute this motion.
     * 
     * @param deltaTimeSeconds The time elapsed since the last update, in seconds
     */
    public default void update(double deltaTimeSeconds) {
    }

    /**
     * Logs telemetry data for this motion.
     * 
     * <p>Implementations should log relevant state information such as target
     * velocities, positions, or motion type for debugging and visualization.
     * 
     * @param basePath The base path in the telemetry hierarchy for this motion's data
     */
    public default void logTelemetry(String basePath) {
    }

    /**
     * Attempts to fuse this motion with another motion.
     * 
     * <p>Fusion allows combining compatible motions, such as merging a translation-only
     * velocity motion with a rotation-only velocity motion into a single combined motion.
     * 
     * <p>The default implementation throws an exception, indicating that fusion is not
     * supported. Subclasses that support fusion should override this method.
     * 
     * @param other The motion to fuse with this one
     * @return A new fused motion combining both motions
     * @throws IllegalArgumentException If the motions cannot be fused
     */
    public default SwerveMotion fuseWith(SwerveMotion other) {
        throw new IllegalArgumentException("Cannot fuse a " + getClass().getSimpleName() + " with a " + other.getClass().getSimpleName());
    }

    /**
     * Releases any resources held by this motion.
     * 
     * <p>Called when this motion is replaced by another motion or when the
     * swerve drive is closed. The default implementation does nothing.
     */
    @Override
    public default void close() {
    }
}
