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

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.math.kinematics.ChassisState;
import org.littletonrobotics.junction.Logger;

public class ChoreoOpenLoopTrajectoryFollower implements TrajectoryFollower {
    Trajectory<SwerveSample> trajectory;
    private double startTime;
    private boolean isFinished = false;

    public ChoreoOpenLoopTrajectoryFollower(String trajectoryName) {
        var trajectoryOptional = Choreo.loadTrajectory(trajectoryName);

        if (trajectoryOptional.isPresent()) {
            try {
                // TODO check if there is a better way to do this
                trajectory = (Trajectory<SwerveSample>) trajectoryOptional.get();
            } catch (ClassCastException e) {
                System.err.println("Trajectory " + trajectoryName + " is not a Swerve trajectory");
            }
        } else {
            System.err.println("Trajectory " + trajectoryName + " not found");
        }
    }

    /** Record the starting time of the trajectory */
    @Override
    public void recordStartingTime() {
        isFinished = false;
        startTime = Logger.getTimestamp() / 1e6;
    }
    ;

    /** {@inheritDoc} */
    public Pose2d getStartPose() {
        SwerveSample sample = trajectory.sampleAt(
                0,
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);
        return new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
    }

    /** {@inheritDoc} */
    @Override
    public ChassisState getState(Pose2d currentPose, ChassisSpeeds currentSpeed) {
        double timestamp = Logger.getTimestamp() / 1e6;
        // Sample the trajectory based on time
        SwerveSample sample = trajectory.sampleAt(
                timestamp - startTime,
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red);

        if (timestamp - startTime > trajectory.getTotalTime()) {
            isFinished = true;
        }

        // compute chassis speeds
        return ChassisState.fromFieldRelativeState(
                sample.vx, sample.vy, sample.omega, sample.ax, sample.ay, sample.alpha, currentPose.getRotation());
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
