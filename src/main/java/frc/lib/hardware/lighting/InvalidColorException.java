package frc.lib.hardware.lighting;

/**
 * Exception thrown when a color is invalid.
 * This can happen when a hex string is not 6 characters long.
 * Or when the hue, saturation, or value is out of range.
 */
public class InvalidColorException extends Exception {
    public InvalidColorException(String message) {
        super(message);
    }
}
