package frc.lib.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A vision measurement
 */
public record VisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs) {

    /**
     * Creates a new {@code VisionMeasurement} object.
     *
     * @param robotPose The robot's pose.
     * @param timestamp The timestamp of the measurement.
     * @param stdDevs The standard deviations of the measurement.
     */
    public VisionMeasurement {}
}
