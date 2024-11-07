package frc.lib.control;

/**
 * Container class for Proportional, Integral, Derivative and FeedForward gains.
 */
public record PIDFGains(PIDGains pid, FeedForwardGains ff) {

    /**
     * Creates an empty {@code PIDFGains} object without gains.
     */
    public PIDFGains() {
        this(new PIDGains(), new FeedForwardGains());
    }

    /**
     * Creates a new {@code PIDFGains} object.
     *
     * @param pid Proportional, Integral, and Derivative gains.
     * @param ff FeedForward gains.
     */
    public PIDFGains {}
}
