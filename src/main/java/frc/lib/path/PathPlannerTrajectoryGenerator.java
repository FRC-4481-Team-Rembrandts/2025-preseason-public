package frc.lib.path;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class PathPlannerTrajectoryGenerator implements TrajectoryGenerator{
    private String pathName;
    private PathPlannerPath path;

    /**
     * Setting up the PathName
     * @param pathName Name of the PathPlanner Path
     */
    public PathPlannerTrajectoryGenerator(String pathName){
        this.pathName = pathName;

    }

    /**
     * Setting the trajectory based on the path from PathPlanner, current Robot Pose
     * and the field relative speeds of the robot
     * @param currentPose the current robot pose
     * @param currentSpeeds the current field relative speeds of the robot
     * @return the desired trajectory
     */
    @Override
    public PathPlannerTrajectory generate(Pose2d currentPose,
                               ChassisSpeeds currentSpeeds) {
        //TODO @Casper, dit moet de robot positie gebruiken. niet aleen de rotatie
        path = PathPlannerPath.fromPathFile(pathName);
        return new PathPlannerTrajectory(path, currentSpeeds, currentPose.getRotation());
    }
}
