/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

/** Parameters to filter vision measurements */
public record VisionFilterParameters(
        double xyStandardDevBase,
        double rotStandardDevBase,
        Distance aprilTagWidth,
        double maxAmbiguityRatio,
        Rotation2d estimatedFOV,
        Distance fieldWidth,
        Distance fieldLength,
        Distance zMargin) {

    /**
     * Create a new set of parameters to filter vision measurements
     *
     * @param xyStandardDevBase The base value for the translational standard deviation
     * @param rotStandardDevBase The base value for the rotational standard deviation
     * @param aprilTagWidth Width of the april tag as {@code Measure<Distance>}
     * @param maxAmbiguityRatio The maximum ambiguity ratio for a target to be considered valid
     * @param estimatedFOV Estimated horizontal field of view of camera as {@code Rotation2d}
     * @param fieldWidth Width of the field as {@code Measure<Distance>}
     * @param fieldLength Length of the field as {@code Measure<Distance>}
     * @param zMargin Margin in Z direction (height) outside which selected poses are not considered valid
     */
    public VisionFilterParameters {}
}
