package frc.lib.control;

/**
 * Container class for FeedForward gains.
 */
public record FeedForwardGains(double kS, double kV, double kA) {

        /**
         * Creates an empty {@code FeedForwardGains} object without gains.
         */
        public FeedForwardGains() {
            this(0,0,0);
        }

        /**
         * Creates a new {@code FeedForwardGains} object.
         *
         * @param kS Static gain.
         * @param kV Velocity gain.
         * @param kA Acceleration gain.
         */
        public FeedForwardGains {}
}
