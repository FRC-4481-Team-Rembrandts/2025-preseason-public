package frc.lib.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.BooleanSupplier;

public interface TrajectoryFollower {

    /**
     * Set the targetSpeed based on the currentPose and currentSpeed
     * @param currentPose The current robot pose
     * @param currentSpeed The current field relative speeds of the robot
     * @return targetSpeeds
     */
    ChassisSpeeds getSpeeds(Pose2d currentPose, ChassisSpeeds currentSpeed);

    /**
     * Returns true if the robot has reached the final destination.
     * @return Whether the robot has reached the final destination.
     */
    boolean isFinished();
}
