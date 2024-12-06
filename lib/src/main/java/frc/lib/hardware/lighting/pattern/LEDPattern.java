/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.hardware.lighting.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.lib.hardware.lighting.LEDStrip;

/**
 * A pattern that can be applied to an LED strip.
 *
 * @see frc.lib.hardware.lighting.LEDStrip
 */
public interface LEDPattern {

    /**
     * Updates the LED buffer with the pattern.
     *
     * @param buffer The LED buffer to update.
     * @param strip The strip to update the buffer with.
     */
    void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip);

    /**
     * Gets the name of the pattern.
     *
     * @return The name of the pattern.
     */
    String getName();
}
