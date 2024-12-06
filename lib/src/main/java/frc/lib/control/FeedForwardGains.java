/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.control;

/** Container class for FeedForward gains. */
public record FeedForwardGains(double kS, double kV, double kA) {

    /** Creates an empty {@code FeedForwardGains} object without gains. */
    public FeedForwardGains() {
        this(0, 0, 0);
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
