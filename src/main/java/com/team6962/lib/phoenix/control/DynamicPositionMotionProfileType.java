package com.team6962.lib.phoenix.control;

/**
 * Enum representing different shapes of motion profiles that can be used for
 * position control with dynamic constraints.
 */
public enum DynamicPositionMotionProfileType {
    /**
     * Trapezoidal motion profile shape, characterized by constant acceleration
     * to a peak velocity, cruising at that velocity, and then constant
     * deceleration to the target velocity.
     */
    Trapezoidal,
    /**
     * Exponential motion profile shape, defined by solving the differential
     * equation:
     * <pre>
     * 12 = kv * dx/dt + ka * d²x/dt²
     * </pre>
     */
    Exponential
}