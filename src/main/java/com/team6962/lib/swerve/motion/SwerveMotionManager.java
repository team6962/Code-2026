package com.team6962.lib.swerve.motion;

public class SwerveMotionManager implements AutoCloseable {
    private SwerveMotion motion = null;

    public synchronized void clear() {
        if (motion != null) {
            motion.close();
        }

        motion = null;
    }

    public synchronized void apply(SwerveMotion newMotion) {
        if (motion != null) {
            motion.close();
        }

        if (motion == null) {
            motion = newMotion;
        } else if (newMotion == null) {
            motion = newMotion;
        } else {
            SwerveMotion fusedMotion = motion.fuseWith(newMotion);

            if (fusedMotion != null) {
                motion = fusedMotion;
            } else {
                motion = newMotion;
            }
        }
    }

    public synchronized SwerveMotion getMotion() {
        return motion;
    }

    @Override
    public synchronized void close() {
        if (motion != null) {
            motion.close();
        }
    }
}
