package com.team6962.lib.phoenix.control;

/**
 * Enum representing different units of output for motor control.
 */
public enum ControlOutputType {
    /**
     * Output in volts, typically in the range of -12V to +12V. Trapezoidal
     * commutation is used.
     */
    Voltage,
    /**
     * Output in volts, typically in the range of -12V to +12V. Field oriented
     * control is used instead of trapezoidal commutation.
     */
    VoltageFOC,
    /**
     * Output in torque current (amps), typically in the range of -120A to
     * +120A. Field oriented control is used instead of trapezoidal commutation.
     */
    TorqueCurrentFOC
}
