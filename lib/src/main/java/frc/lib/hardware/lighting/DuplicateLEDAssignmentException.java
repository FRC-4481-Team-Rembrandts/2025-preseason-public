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
 * Exception thrown when a duplicate LED assignment is attempted. This can happen when two LED strips are assigned to
 * the same LEDs.
 */
public class DuplicateLEDAssignmentException extends Exception {
    public DuplicateLEDAssignmentException(String message) {
        super(message);
    }
}
