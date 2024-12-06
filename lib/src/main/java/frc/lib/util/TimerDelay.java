/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * A class that helps with delaying code execution. This class is intended to simulate a delay without actually stopping
 * the code.
 */
public class TimerDelay {
    boolean lock;
    double startTimeStamp;

    /** Creates a new TimerDelay object. */
    public TimerDelay() {
        reset();
    }

    /**
     * Sets a delay for the specified amount of seconds.
     *
     * @param seconds The amount of seconds to delay for.
     * @return Whether the delayed time has passed.
     */
    public boolean setDelay(double seconds) {
        if (!lock) {
            startTimeStamp = Timer.getFPGATimestamp();
            lock = true;
        }
        return Timer.getFPGATimestamp() - startTimeStamp >= seconds;
    }

    /** Resets the timer. */
    public void reset() {
        lock = false;
    }
}
