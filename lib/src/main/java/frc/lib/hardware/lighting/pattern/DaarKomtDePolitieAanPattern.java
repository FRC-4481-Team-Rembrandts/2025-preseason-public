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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.hardware.lighting.Color;
import frc.lib.hardware.lighting.LEDStrip;
import frc.lib.util.TimerDelay;

/**
 * A blinking pattern that can be applied to an LED strip. This pattern will alternate between both alliance colors and
 * finally land on the color of the current alliance.
 *
 * @see frc.lib.hardware.lighting.LEDStrip
 * @see <a href="https://www.youtube.com/watch?v=v1RGlLtWuH8">Daar komt de politie aan</a>
 */
public class DaarKomtDePolitieAanPattern implements LEDPattern {

    private Color.RGB redColor = new Color.RGB(255, 0, 0);
    private Color.RGB blueColor = new Color.RGB(0, 0, 255);
    private final TimerDelay solidDelay = new TimerDelay();
    private int microsecondFlip = 300000;

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
        int halfLength = length / 2;

        Color.RGB firstColor = redColor;
        Color.RGB secondColor = blueColor;

        // Switching states should be done twice per duration
        if (solidDelay.setDelay(3)) {
            // After two seconds stop blinking
            if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                firstColor = redColor;
                secondColor = redColor;
            } else {
                firstColor = blueColor;
                secondColor = blueColor;
            }
        } else if (RobotController.getFPGATime() % microsecondFlip * 2 < microsecondFlip) {
            // If two seconds have not passed
            firstColor = redColor;
            secondColor = blueColor;
        } else {
            firstColor = blueColor;
            secondColor = redColor;
        }

        // Assign color to first half of ledstrip
        for (int i = 0; i < halfLength; i++) {
            buffer.setRGB(offset + i, firstColor.red(), firstColor.green(), firstColor.blue());
        }

        // Assign color to second half of ledstrip
        for (int i = halfLength; i < length; i++) {
            buffer.setRGB(offset + i, secondColor.red(), secondColor.green(), secondColor.blue());
        }
    }

    @Override
    public String getName() {
        return "DaarKomtDePolitieAanPattern";
    }
}
