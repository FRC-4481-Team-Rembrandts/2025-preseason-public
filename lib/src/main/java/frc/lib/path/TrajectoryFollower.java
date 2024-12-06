/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.math.kinematics.ChassisState;

/**
 * Interface to define a trajectory follower. Implementations of this interface should include two important things.
 * First of an algorithm to drive to a certain sample of a trajectory and secondly a way to generate this trajectory,
 * this will generally be done in the constructor.
 */
public interface TrajectoryFollower {

    /** Record the starting time of the trajectory */
    void recordStartingTime();

    /** Get the first pose of the trajectory, this is used to reset the drivetrain odometry */
    Pose2d getStartPose();

    /**
     * Retrieve the target drivetrain state based on the currentPose and currentSpeed
     *
     * @param currentPose The current robot pose
     * @param currentSpeed The current field relative speeds of the robot
     * @return The target ChassisState that the drivetrain should achieve to follow the trajectory
     */
    ChassisState getState(Pose2d currentPose, ChassisSpeeds currentSpeed);

    /**
     * Returns true if the robot has reached the final destination.
     *
     * @return Whether the robot has reached the final destination.
     */
    boolean isFinished();
}
