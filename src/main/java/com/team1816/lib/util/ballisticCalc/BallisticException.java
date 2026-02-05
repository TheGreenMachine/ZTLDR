package com.team1816.lib.util.ballisticCalc;

/**
 * Exception thrown when a ballistic calculation cannot be completed.
 */
public class BallisticException extends RuntimeException {

    public BallisticException(String message) {
        super(message);
    }

    public BallisticException(String message, Throwable cause) {
        super(message, cause);
    }
}
