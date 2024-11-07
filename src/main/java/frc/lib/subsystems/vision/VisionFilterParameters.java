package frc.lib.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

/**
 * Parameters to filter vision measurements
 */
public record VisionFilterParameters(double xyStandardDevBase, double rotStandardDevBase, Measure<Distance> aprilTagWidth,
                                     double maxAmbiguityRatio, Rotation2d estimatedFOV,
                                     Measure<Distance> fieldWidth, Measure<Distance> fieldLength, Measure<Distance> zMargin) {

    /**
     * Create a new set of parameters to filter vision measurements
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
