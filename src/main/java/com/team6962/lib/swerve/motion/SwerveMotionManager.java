package com.team6962.lib.swerve.motion;

public class SwerveMotionManager implements AutoCloseable {
    private SwerveMotion nextMotion = null;
    private SwerveMotion activeMotion;
    private SwerveMotion defaultMotion;

    public SwerveMotionManager(SwerveMotion defaultMotion) {
        this.defaultMotion = defaultMotion;
    }

    public synchronized void update() {
        if (activeMotion != null) {
            activeMotion.close();
        }

        activeMotion = nextMotion;
        nextMotion = null;
    }

    public synchronized void apply(SwerveMotion newMotion) {
        if (nextMotion != null) {
            nextMotion.close();
        }

        if (nextMotion == null || newMotion == null) {
            nextMotion = newMotion;
        } else {
            SwerveMotion fusedMotion = nextMotion.fuseWith(newMotion);

            if (fusedMotion != null) {
                nextMotion = fusedMotion;
            } else {
                nextMotion = newMotion;
            }
        }
    }

    public synchronized SwerveMotion getActiveMotion() {
        if (activeMotion != null) {
            return activeMotion;
        } else {
            return defaultMotion;
        }
    }

    @Override
    public synchronized void close() {
        if (nextMotion != null) {
            nextMotion.close();
        }
    }
}
