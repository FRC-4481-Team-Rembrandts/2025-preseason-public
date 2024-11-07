package frc.lib.control;

/**
 * Container class for Proportional, Integral, and Derivative gains.
 */
public record PIDGains(double kP, double kI, double kD) {

    /**
     * Creates an empty {@code PIDGains} object without gains.
     */
    public PIDGains() {
        this(0,0,0);
    }

    /**
     * Creates a new {@code PIDGains} object.
     *
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     */
    public PIDGains {}
}
