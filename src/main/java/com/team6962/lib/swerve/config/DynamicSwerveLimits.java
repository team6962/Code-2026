package com.team6962.lib.swerve.config;

import com.team6962.lib.phoenix.control.VelocityMotionProfileType;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class DynamicSwerveLimits {
  private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();

  private LinearVelocity maxLinearVelocity;
  private VelocityMotionProfileType motionProfileType;

  public DynamicSwerveLimits(
      LinearVelocity maxLinearVelocity, VelocityMotionProfileType motionProfileType) {
    this.maxLinearVelocity = maxLinearVelocity;
    this.motionProfileType = motionProfileType;
  }

  public void update(
      LinearVelocity maxLinearVelocity, VelocityMotionProfileType motionProfileType) {
    lock.writeLock().lock();
    try {
      this.maxLinearVelocity = maxLinearVelocity;
      this.motionProfileType = motionProfileType;
    } finally {
      lock.writeLock().unlock();
    }
  }

  public LinearVelocity getMaxLinearVelocity() {
    lock.readLock().lock();
    try {
      return maxLinearVelocity;
    } finally {
      lock.readLock().unlock();
    }
  }

  public VelocityMotionProfileType getMotionProfileType() {
    lock.readLock().lock();
    try {
      return motionProfileType;
    } finally {
      lock.readLock().unlock();
    }
  }
}
