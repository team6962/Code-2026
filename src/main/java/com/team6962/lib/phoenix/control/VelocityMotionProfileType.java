package com.team6962.lib.phoenix.control;

/**
 * Enum representing different shapes of motion profiles that can be used for
 * velocity control.
 */
public enum VelocityMotionProfileType {
    /**
     * No motion profiling is applied; only PID, static friction feedforward,
     * gravity feedforward, and velocity feedforward are used.
     */
    None,
    /**
     * Trapezoidal motion profile shape, characterized by constant acceleration
     * to a peak velocity, cruising at that velocity, and then constant
     * deceleration to the target velocity.
     */
    Trapezoidal
}
