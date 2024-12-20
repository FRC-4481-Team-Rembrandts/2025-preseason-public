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
 * A rainbow pattern that can be applied to an LED strip. This pattern will override the hue of the primary color. The
 * saturation and value of the primary color are applied to the rainbow. The secondary color is not used.
 *
 * @see frc.lib.hardware.lighting.LEDStrip
 */
public class RainbowPattern implements LEDPattern {
    // Constants
    private static final int HUE_RANGE = 180;

    private int rainbowFirstPixelHue = 0;
    private double hueShiftBuffer = 0;
    private double previousTime = Timer.getFPGATimestamp();

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
        double patternSeconds = strip.getPatternDuration();
        Color.HSV color = strip.getPrimaryColor().getHSV();

        // Calculate delta time
        double newTime = Timer.getFPGATimestamp();
        double deltaTime = newTime - previousTime;
        previousTime = newTime;

        // For every pixel
        for (int i = 0; i < length; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final int hue = (rainbowFirstPixelHue + (i * HUE_RANGE / length)) % HUE_RANGE;
            // Set the value
            buffer.setHSV(i + offset, hue, color.saturation(), color.value());
        }

        // Increase by to make the rainbow "move"
        hueShiftBuffer += ((deltaTime / patternSeconds) * HUE_RANGE);
        int hueShift = (int) hueShiftBuffer;
        hueShiftBuffer -= hueShift;
        rainbowFirstPixelHue += hueShift;

        // Check bounds
        rainbowFirstPixelHue %= HUE_RANGE;
        if (rainbowFirstPixelHue < 0) rainbowFirstPixelHue += HUE_RANGE;
    }

    @Override
    public String getName() {
        return "Rainbow";
    }
}
