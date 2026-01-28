package com.team6962.lib.swerve.motion;

/**
 * Manages the lifecycle of swerve motions, and fuses translational and rotational motions together
 * before applying them to control the swerve drive.
 *
 * <p>The SwerveMotionManager handles the coordination of motion commands sent to the swerve drive.
 * It implements a buffering pattern where:
 *
 * <ol>
 *   <li>New motions are queued as the "next" motion via {@link #apply(SwerveMotion)}
 *   <li>During {@link #update()}, the next motion becomes the active motion, and the next motion is
 *       cleared so it won't be fused with later motions
 *   <li>The active motion is executed each control loop iteration
 * </ol>
 *
 * <p>This design ensures thread-safe motion transitions and allows motions to be applied from any
 * thread while updates occur in the control loop thread.
 *
 * <p>When motions are applied in sequence (before an update), they are fused together using {@link
 * SwerveMotion#fuseWith(SwerveMotion)}.
 *
 * <p>A default motion must be provided at construction, which is returned by {@link
 * #getActiveMotion()} when no other motion is active.
 *
 * @see SwerveMotion
 */
public class SwerveMotionManager implements AutoCloseable {
  /** The motion queued to become active on the next update. */
  private SwerveMotion nextMotion = null;

  /** The currently executing motion. */
  private SwerveMotion activeMotion;

  /** The fallback motion used when no active motion is set. */
  private SwerveMotion defaultMotion;

  /**
   * Creates a new SwerveMotionManager with the specified default motion.
   *
   * @param defaultMotion The motion to use when no other motion is active
   */
  public SwerveMotionManager(SwerveMotion defaultMotion) {
    this.defaultMotion = defaultMotion;
  }

  /**
   * Transitions to the next queued motion.
   *
   * <p>This method should be called once per control loop iteration. It closes the current active
   * motion and promotes the next motion to active status. After this call, the next motion slot is
   * cleared.
   */
  public synchronized void update() {
    if (activeMotion != null) {
      activeMotion.close();
    }

    activeMotion = nextMotion;
    nextMotion = null;
  }

  /**
   * Queues a new motion to be applied on the next update.
   *
   * <p>If a motion is already queued, the two motions are fused together using {@link
   * SwerveMotion#fuseWith(SwerveMotion)}.
   *
   * @param newMotion The motion to apply, or {@code null} to apply the default motion
   */
  public synchronized void apply(SwerveMotion newMotion) {
    if (nextMotion != null) {
      nextMotion.close();
    }

    if (nextMotion == null || newMotion == null) {
      nextMotion = newMotion;
    } else {
      nextMotion = nextMotion.fuseWith(newMotion);
    }
  }

  /**
   * Gets the current active motion.
   *
   * <p>Returns the active motion if one is set, otherwise returns the default motion.
   *
   * @return The active motion, or the default motion if no active motion is set
   */
  public synchronized SwerveMotion getActiveMotion() {
    if (activeMotion != null) {
      return activeMotion;
    } else {
      return defaultMotion;
    }
  }

  /** Closes the motion manager and any enclosed motions. */
  @Override
  public synchronized void close() {
    if (nextMotion != null) {
      nextMotion.close();
    }

    if (activeMotion != null) {
      activeMotion.close();
    }
  }
}
