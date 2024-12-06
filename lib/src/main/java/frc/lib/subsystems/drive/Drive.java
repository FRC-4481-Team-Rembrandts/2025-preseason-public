/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.kinematics.ChassisState;
import frc.lib.path.TrajectoryFollower;
import frc.lib.subsystems.vision.VisionMeasurement;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A subsystem that represents a drivetrain. Can be used to create commands that control the drivetrain without needing
 * to know the specifics.
 */
public abstract class Drive extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();

    /**
     * Create a new drive
     *
     * @param odometryLooperFrequency The looper periodic time in seconds of the odometry thread
     */
    public Drive(double odometryLooperFrequency) {
        super();
        // Start threads (no-op if no signals have been created)
        SparkOdometryThread.getInstance().start(1.0 / odometryLooperFrequency);
    }

    /**
     * Sets the target speeds for the drivetrain. Speeds should be robot relative.
     *
     * @param targetSpeeds The target speeds for the drivetrain.
     */
    protected abstract void setTargetSpeeds(ChassisSpeeds targetSpeeds);

    /**
     * Sets the target chassis state (velocity plus acceleration) for the drivetrain. The ChassisState should be
     * robotrelative
     *
     * @param targetState The target state for the drivetrain.
     */
    protected abstract void setTargetState(ChassisState targetState);

    /**
     * Gets the robot relative speeds of the drivetrain.
     *
     * @return The robot relative speeds of the drivetrain.
     */
    protected abstract ChassisSpeeds getRobotRelativeSpeeds();

    /**
     * Gets the field relative speeds of the drivetrain.
     *
     * @return The field relative speeds of the drivetrain.
     */
    protected abstract ChassisSpeeds getFieldRelativeSpeeds();

    /**
     * Gets the pose of the drivetrain.
     *
     * @return The pose of the drivetrain.
     */
    protected abstract Pose2d getPose();

    /**
     * Gets the maximum velocity of the drivetrain.
     *
     * @return The maximum velocity of the drivetrain.
     */
    protected abstract double getMaxVelocity();

    /**
     * Gets the maximum angular velocity of the drivetrain.
     *
     * @return The maximum angular velocity of the drivetrain.
     */
    protected abstract double getMaxAngularVelocity();

    /**
     * Gets the maximum acceleration of the drivetrain.
     *
     * @return The maximum acceleration of the drivetrain.
     */
    protected abstract double getMaxAcceleration();

    /**
     * Gets the maximum angular acceleration of the drivetrain.
     *
     * @return The maximum angular acceleration of the drivetrain.
     */
    protected abstract double getMaxAngularAcceleration();

    /**
     * Get the supplier for the current robot pose
     *
     * @return The supplier for the current robot pose
     */
    public abstract Supplier<Pose2d> getRobotPoseSupplier();

    /**
     * Get the consumer for the vision measurements, this is used by the vision subsystem to pass measurements to the
     * drive subsystem
     *
     * @return The consumer for the vision measurements
     */
    public abstract Consumer<VisionMeasurement> getVisionMeasurementConsumer();

    /**
     * Get the supplier for the field relative speeds
     *
     * @return The supplier for the field relative speeds
     */
    public abstract Supplier<ChassisSpeeds> getFieldRelativeSpeedsSupplier();

    /** Field relative drive command using two joysticks (controlling linear and angular velocities). */
    public abstract Command joystickDrive(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, double deadband);

    /**
     * Sets the robot to a specified pose.
     *
     * @param poseSupplier The pose to reset to.
     */
    public abstract Command setRobotPose(Supplier<Pose2d> poseSupplier);

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     *
     * @param trajectoryFollower a trajectoryFollower.
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
    public Command followTrajectory(TrajectoryFollower trajectoryFollower) {
        return Commands.sequence(
                Commands.runOnce(trajectoryFollower::recordStartingTime),
                setTargetStateCommand(() -> trajectoryFollower.getState(this.getPose(), this.getFieldRelativeSpeeds()))
                        .until(trajectoryFollower::isFinished));
    }

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     *
     * @param trajectoryFollower a trajectoryFollower.
     * @param resetToStartPosition Whether the position of the robot should be reset to the starting position of the
     *     trajectory. This should only be done for the first trajectory in autonomous when there is no vision.
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
    public Command followTrajectory(TrajectoryFollower trajectoryFollower, boolean resetToStartPosition) {
        if (!resetToStartPosition) {
            return followTrajectory(trajectoryFollower);
        }
        return Commands.sequence(setRobotPose(trajectoryFollower::getStartPose), followTrajectory(trajectoryFollower));
    }

    /**
     * Continuously sets the target speeds for the drivetrain. Speeds should be robot relative.
     *
     * @param chassisSpeeds Supplier of robot relative ChassisSpeed
     */
    private Command setTargetSpeedsCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
        return Commands.run(() -> setTargetSpeeds(chassisSpeeds.get()), this);
    }

    /**
     * Continuously sets the target speeds for the drivetrain. Speeds should be robot relative.
     *
     * @param xSpeed Supplier of the target speed in the x direction.
     * @param ySpeed Supplier of the target speed in the y direction.
     * @param angularSpeed Supplier of the target angular speed.
     */
    private Command setTargetSpeedsCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier angularSpeed) {
        return Commands.run(
                () -> setTargetSpeeds(
                        new ChassisSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), angularSpeed.getAsDouble())),
                this);
    }

    /**
     * Continuously sets the target speeds for the drivetrain. Speeds should be robot relative. To be used for
     * non-holonomic drivetrains.
     *
     * @param xSpeed Supplier of the target speed in the x direction.
     * @param angularSpeed Supplier of the target angular speed.
     */
    private Command setTargetSpeedsCommand(DoubleSupplier xSpeed, DoubleSupplier angularSpeed) {
        return Commands.run(
                () -> setTargetSpeeds(new ChassisSpeeds(xSpeed.getAsDouble(), 0, angularSpeed.getAsDouble())), this);
    }

    /**
     * Continuously sets the target state for the drivetrain. Speeds should be robot relative.
     *
     * @param targetState Supplier of the target state for the drivetrain including accelerations
     */
    private Command setTargetStateCommand(Supplier<ChassisState> targetState) {
        return Commands.run(() -> setTargetState(targetState.get()), this);
    }
}
