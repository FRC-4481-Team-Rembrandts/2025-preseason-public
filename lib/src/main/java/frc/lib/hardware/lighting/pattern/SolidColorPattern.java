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
import frc.lib.hardware.lighting.Color;
import frc.lib.hardware.lighting.LEDStrip;

/**
 * A solid color pattern that can be applied to an LED strip.
 *
 * @see frc.lib.hardware.lighting.LEDStrip
 */
public class SolidColorPattern implements LEDPattern {

    /**
     * Updates the LED buffer with the pattern.
     *
     * @param buffer The LED buffer to update.
     * @param strip The strip to update the buffer with.
     */
    @Override
    public void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip) {
        int offset = strip.getOffset();
        int length = strip.getLength();
        Color.HSV color = strip.getPrimaryColor().getHSV();

        for (int i = 0; i < length; i++) {
            buffer.setHSV(offset + i, color.hue(), color.saturation(), color.value());
        }
    }

    @Override
    public String getName() {
        return "SolidColor";
    }
}
