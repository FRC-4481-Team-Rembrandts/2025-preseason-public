/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.hardware.lighting;

/**
 * Exception thrown when a color is invalid. This can happen when a hex string is not 6 characters long. Or when the
 * hue, saturation, or value is out of range.
 */
public class InvalidColorException extends Exception {
    public InvalidColorException(String message) {
        super(message);
    }
}
