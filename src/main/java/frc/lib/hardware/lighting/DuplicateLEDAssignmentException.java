package frc.lib.hardware.lighting;

/**
 * Exception thrown when a duplicate LED assignment is attempted.
 * This can happen when two LED strips are assigned to the same LEDs.
 */
public class DuplicateLEDAssignmentException extends Exception {
    public DuplicateLEDAssignmentException(String message) {
        super(message);
    }
}
