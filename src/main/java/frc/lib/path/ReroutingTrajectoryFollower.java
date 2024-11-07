package frc.lib.path;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.Optional;
import java.util.function.BooleanSupplier;

public class ReroutingTrajectoryFollower implements TrajectoryFollower{
    private final TrajectoryGenerator trajectoryGenerator;
    private PathPlannerTrajectory trajectory;
    private Optional<Double> startTime = Optional.empty();
    private boolean isFinished = false;

    /**
     * Setting the trajectory
     * @param trajectoryGenerator The chosen trajectoryGenerator
     * @param currentPose The current Pose
     * @param currentSpeeds The current Speeds
     */

    public ReroutingTrajectoryFollower(TrajectoryGenerator trajectoryGenerator,
                                       Pose2d currentPose,
                                       ChassisSpeeds currentSpeeds){
        this.trajectoryGenerator = trajectoryGenerator;
        trajectory = trajectoryGenerator.generate(currentPose, currentSpeeds);
    }

    /**
     * Reroutes the trajectory if there occurs a large error
     * @param currentPose The current pose
     * @param currentSpeed The current Speeds
     * @return TargetSpeeds
     */
    @Override
    public ChassisSpeeds getSpeeds(Pose2d currentPose, ChassisSpeeds currentSpeed) {
        // First time method is called initialize startTime
        if (startTime.isEmpty()) {
            startTime = Optional.of(MathSharedStore.getTimestamp());
        }
        double timestamp = MathSharedStore.getTimestamp();
        //Sample the trajectory based on time
        PathPlannerTrajectory.State targetState = trajectory.sample(timestamp - startTime.get());
        //Calculate the distance of the robot to this sampled point
        double distanceToPoint = currentPose.getTranslation().getDistance(targetState.positionMeters);
        //Check if the distance to the target position is larger than THRESHOLD
        //TODO add threshold
//        if (distanceToPoint > 1){
//            // regenerate the trajectory from the currentPose, but start the trajectory from the last sampledState
//            trajectory = trajectoryGenerator.generate(currentPose, currentSpeed);
//            // resample state
//            targetState = trajectory.sample(timestamp - startTime);
//        }

        // check isFinished boolean
        if (timestamp - startTime.get() > trajectory.getTotalTimeSeconds()) {
            isFinished = true;
        }

        // compute chassis speeds
        double xFF = targetState.velocityMps * targetState.heading.getCos();
        double yFF = targetState.velocityMps * targetState.heading.getSin();
        double rotationFF = targetState.holonomicAngularVelocityRps.orElse(0.0);

        return new ChassisSpeeds(xFF, yFF, rotationFF);
    }

    /**
     * Returns true if the robot has reached the final destination.
     * @return Whether the robot has reached the final destination.
     */
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}