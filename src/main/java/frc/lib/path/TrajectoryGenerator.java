package frc.lib.path;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

@FunctionalInterface
public interface TrajectoryGenerator {

     /**
      * generates a PathPlanner Trajectory based on the current pose and speed of the robot
      * @param robotPose the current robot pose
      * @param currentSpeeds the current field relative speeds of the robot
      * @return the desired trajectory
      */
     PathPlannerTrajectory generate(Pose2d robotPose,
                                    ChassisSpeeds currentSpeeds);
}
