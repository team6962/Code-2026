package com.team6962.lib.logging;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

class LoggingUtilTest {

    // ==================== getDouble() tests ====================

    @Test
    void getDouble_ValidInteger_ReturnsDouble() {
        Optional<Double> result = LoggingUtil.getDouble("42");
        assertTrue(result.isPresent());
        assertEquals(42.0, result.get(), 1e-9);
    }

    @Test
    void getDouble_ValidDecimal_ReturnsDouble() {
        Optional<Double> result = LoggingUtil.getDouble("3.14159");
        assertTrue(result.isPresent());
        assertEquals(3.14159, result.get(), 1e-9);
    }

    @Test
    void getDouble_NegativeNumber_ReturnsDouble() {
        Optional<Double> result = LoggingUtil.getDouble("-7.5");
        assertTrue(result.isPresent());
        assertEquals(-7.5, result.get(), 1e-9);
    }

    @Test
    void getDouble_Zero_ReturnsDouble() {
        Optional<Double> result = LoggingUtil.getDouble("0");
        assertTrue(result.isPresent());
        assertEquals(0.0, result.get(), 1e-9);
    }

    @Test
    void getDouble_ScientificNotation_ReturnsDouble() {
        Optional<Double> result = LoggingUtil.getDouble("1.5e3");
        assertTrue(result.isPresent());
        assertEquals(1500.0, result.get(), 1e-9);
    }

    @Test
    void getDouble_NegativeScientificNotation_ReturnsDouble() {
        Optional<Double> result = LoggingUtil.getDouble("-2.5e-2");
        assertTrue(result.isPresent());
        assertEquals(-0.025, result.get(), 1e-9);
    }

    @Test
    void getDouble_InvalidString_ReturnsEmpty() {
        Optional<Double> result = LoggingUtil.getDouble("not a number");
        assertFalse(result.isPresent());
    }

    @Test
    void getDouble_EmptyString_ReturnsEmpty() {
        Optional<Double> result = LoggingUtil.getDouble("");
        assertFalse(result.isPresent());
    }

    @Test
    void getDouble_MixedContent_ReturnsEmpty() {
        Optional<Double> result = LoggingUtil.getDouble("123abc");
        assertFalse(result.isPresent());
    }

    @Test
    void getDouble_WhitespaceOnly_ReturnsEmpty() {
        Optional<Double> result = LoggingUtil.getDouble("   ");
        assertFalse(result.isPresent());
    }

    @Test
    void getDouble_LeadingWhitespace_ReturnsDouble() {
        Optional<Double> result = LoggingUtil.getDouble("  5.0");
        assertTrue(result.isPresent());
        assertEquals(5.0, result.get(), 1e-9);
    }

    @Test
    void getDouble_TrailingWhitespace_ReturnsDouble() {
        Optional<Double> result = LoggingUtil.getDouble("5.0  ");
        assertTrue(result.isPresent());
        assertEquals(5.0, result.get(), 1e-9);
    }

    // ==================== ensureEndsWithSlash() tests ====================

    @Test
    void ensureEndsWithSlash_NoSlash_AddsSlash() {
        String result = LoggingUtil.ensureEndsWithSlash("path/to/something");
        assertEquals("path/to/something/", result);
    }

    @Test
    void ensureEndsWithSlash_AlreadyHasSlash_NoChange() {
        String result = LoggingUtil.ensureEndsWithSlash("path/to/something/");
        assertEquals("path/to/something/", result);
    }

    @Test
    void ensureEndsWithSlash_EmptyString_AddsSlash() {
        String result = LoggingUtil.ensureEndsWithSlash("");
        assertEquals("/", result);
    }

    @Test
    void ensureEndsWithSlash_JustSlash_NoChange() {
        String result = LoggingUtil.ensureEndsWithSlash("/");
        assertEquals("/", result);
    }

    @Test
    void ensureEndsWithSlash_SingleWord_AddsSlash() {
        String result = LoggingUtil.ensureEndsWithSlash("Drivetrain");
        assertEquals("Drivetrain/", result);
    }

    @Test
    void ensureEndsWithSlash_MultipleTrailingSlashes_NoChange() {
        String result = LoggingUtil.ensureEndsWithSlash("path//");
        assertEquals("path//", result);
    }

    @Test
    void ensureEndsWithSlash_LeadingSlash_AddsTrailingSlash() {
        String result = LoggingUtil.ensureEndsWithSlash("/path/to/something");
        assertEquals("/path/to/something/", result);
    }
}
