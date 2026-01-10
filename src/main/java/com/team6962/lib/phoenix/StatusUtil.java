package com.team6962.lib.phoenix;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * Utility class for interpreting CTRE Phoenix {@link StatusSignal}s and
 * {@link StatusCode}s. This class provides two methods for logging error codes:
 *
 * <ul>
 *   <li>{@link #check(StatusCode)} logs an error with a stack trace if the
 *       status code is not OK
 *   <li>{@link #unwrap(StatusSignal)} returns the value contained in a status
 * 		 signal, logging an error with a stack trace if the status code is not
 *       OK
 * </ul>
 * 
 * Additionally, this class provides the method {@link #toFPGATimestamp(double)}
 * for converting status signal timestamps to FPGA timestamps.
 */
public final class StatusUtil {
	private StatusUtil() {}

	/**
	 * Checks the given {@link StatusCode} and, if it is not OK, prints an error
	 * message and stack trace.
	 * @param code the {@link StatusCode} to check
	 */
	public static void check(StatusCode code) {
		if (code.isOK()) return;

		System.err.println(
            "Status Code " + code.value + " " + code.getName() + ": " +
            code.getDescription()
        );

		for (StackTraceElement element : Thread.currentThread().getStackTrace()) {
		    System.err.println("\n\tat " + element);
		}
	}

	/**
	 * Unwraps the value from the given {@link StatusSignal}, checking the
	 * status code and logging an error with a stack trace if it is not OK.
	 * @param <T> the type of the value contained in the status signal
	 * @param signal the {@link StatusSignal} to unwrap
	 * @return the value contained in the status signal
	 */
	public static <T> T unwrap(StatusSignal<T> signal) {
		check(signal.getStatus());

		return signal.getValue();
	}

    /**
     * Converts a status signal timestamp to an FPGA timestamp in the same
     * timebase as return values of {@link Timer#getFPGATimestamp()}.
     * 
     * @param signalTimestamp the status signal timestamp to convert, in seconds
     * @return the equivalent FPGA timestamp in seconds
     */
	public static double toFPGATimestamp(double signalTimestamp) {
		return signalTimestamp + Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds();
	}
}
