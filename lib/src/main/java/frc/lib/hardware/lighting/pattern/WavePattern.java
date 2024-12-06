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
import edu.wpi.first.wpilibj.Timer;
import frc.lib.hardware.lighting.Color;
import frc.lib.hardware.lighting.LEDStrip;

/**
 * A wave pattern that can be applied to an LED strip. This pattern will interpolate between the primary and secondary
 * colors. These colors will shift over time.
 *
 * @see frc.lib.hardware.lighting.LEDStrip
 */
public class WavePattern implements LEDPattern {
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
        Color.RGB primary = strip.getPrimaryColor().getRGB();
        Color.RGB secondary = strip.getSecondaryColor().getRGB();

        // Fading is done by changing the value of the color over a sine wave
        double ledWavelength = (2 * Math.PI) / length;
        double timeWavelength = (2 * Math.PI) / strip.getPatternDuration();

        // Calculate time offset
        double timeOffset = Timer.getFPGATimestamp() * timeWavelength;

        for (int i = 0; i < length; i++) {
            double interpolation = Math.sin(ledWavelength * i + timeOffset) * 0.5 + 0.5;

            buffer.setRGB(
                    offset + i,
                    interpolate(primary.red(), secondary.red(), interpolation),
                    interpolate(primary.green(), secondary.green(), interpolation),
                    interpolate(primary.blue(), secondary.blue(), interpolation));
        }
    }

    private int interpolate(int start, int end, double interpolationValue) {
        return (int) (start + (end - start) * interpolationValue);
    }

    @Override
    public String getName() {
        return "Wave";
    }
}
