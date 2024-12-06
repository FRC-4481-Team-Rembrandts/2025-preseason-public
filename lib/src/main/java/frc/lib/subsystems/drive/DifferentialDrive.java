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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.math.kinematics.ChassisState;
import frc.lib.subsystems.vision.VisionMeasurement;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DifferentialDrive extends Drive {

    private final double TRACK_WIDTH_METERS;
    private final double MAX_VELOCITY;
    private final double MAX_ACCELERATION;

    private final DifferentialDriveIO leftModuleIO;
    private final DifferentialDriveInputsAutoLogged leftInputs = new DifferentialDriveInputsAutoLogged();
    private double leftPreviousPosition;
    private final DifferentialDriveIO rightModuleIO;
    private final DifferentialDriveInputsAutoLogged rightInputs = new DifferentialDriveInputsAutoLogged();
    private double rightPreviousPosition;

    private Rotation2d robotAngle = new Rotation2d();

    private final DifferentialDriveKinematics kinematics;

    private final DifferentialDrivePoseEstimator poseEstimator;

    private final IMUIO imuIO;
    private final IMUInputsAutoLogged imuInputs = new IMUInputsAutoLogged();

    public final Supplier<Pose2d> robotPoseSupplier;
    public final Consumer<VisionMeasurement> visionMeasurementConsumer;

    public DifferentialDrive(
            double trackWidthMeters,
            double maxVelocity,
            double maxAcceleration,
            double odometryLooperDt,
            IMUIO imuIO,
            DifferentialDriveIO leftModuleIO,
            DifferentialDriveIO rightModuleIO) {
        super(odometryLooperDt);

        TRACK_WIDTH_METERS = trackWidthMeters;
        MAX_VELOCITY = maxVelocity;
        MAX_ACCELERATION = maxAcceleration;

        this.imuIO = imuIO;

        this.leftModuleIO = leftModuleIO;
        this.rightModuleIO = rightModuleIO;

        leftPreviousPosition = leftInputs.position;
        rightPreviousPosition = rightInputs.position;

        kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
        poseEstimator = new DifferentialDrivePoseEstimator(
                kinematics, robotAngle, leftInputs.position, rightInputs.position, new Pose2d());

        // Define the consumers and suppliers
        robotPoseSupplier = poseEstimator::getEstimatedPosition;
        visionMeasurementConsumer = (measurement) -> {
            poseEstimator.addVisionMeasurement(measurement.robotPose(), measurement.timestamp(), measurement.stdDevs());
        };
    }

    @Override
    public void periodic() {
        imuIO.updateInputs(imuInputs);
        Logger.processInputs("DifferentialDrive/IMU", imuInputs);
        leftModuleIO.updateInputs(leftInputs);
        Logger.processInputs("DifferentialDrive/Left", leftInputs);
        rightModuleIO.updateInputs(rightInputs);
        Logger.processInputs("DifferentialDrive/Right", rightInputs);

        if (imuInputs.connected) {
            robotAngle = imuInputs.yawPosition;
        } else {
            // estimation, but better than no data at all
            double leftDeltaPosition = leftInputs.position - leftPreviousPosition;
            double rightDeltaPosition = rightInputs.position - rightPreviousPosition;

            Twist2d twist = kinematics.toTwist2d(leftDeltaPosition, rightDeltaPosition);
            robotAngle = robotAngle.plus(new Rotation2d(twist.dtheta));
        }

        poseEstimator.update(robotAngle, leftInputs.position, rightInputs.position);
        leftPreviousPosition = leftInputs.position;
        rightPreviousPosition = rightInputs.position;
    }

    /**
     * Sets the target speeds for the drivetrain. Speeds should be robot relative.
     *
     * @param targetSpeeds The target speeds for the drivetrain.
     */
    @Override
    protected void setTargetSpeeds(ChassisSpeeds targetSpeeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(targetSpeeds);
        wheelSpeeds.desaturate(getMaxVelocity());
        leftModuleIO.setDriveVelocity(wheelSpeeds.leftMetersPerSecond);
        rightModuleIO.setDriveVelocity(wheelSpeeds.rightMetersPerSecond);
    }

    /** {@inheritDoc} */
    @Override
    protected void setTargetState(ChassisState targetState) {
        setTargetSpeeds(targetState.toChassisSpeeds());
    }

    /**
     * Gets the robot relative speeds of the drivetrain.
     *
     * @return The robot relative speeds of the drivetrain.
     */
    @Override
    protected ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftInputs.velocity, rightInputs.velocity));
    }

    /**
     * Gets the field relative speeds of the drivetrain.
     *
     * @return The field relative speeds of the drivetrain.
     */
    @Override
    protected ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                getRobotRelativeSpeeds(), poseEstimator.getEstimatedPosition().getRotation());
    }

    /**
     * Gets the pose of the drivetrain.
     *
     * @return The pose of the drivetrain.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    protected Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the maximum velocity of the drivetrain.
     *
     * @return The maximum velocity of the drivetrain.
     */
    @Override
    protected double getMaxVelocity() {
        return MAX_VELOCITY;
    }

    /**
     * Gets the maximum angular velocity of the drivetrain.
     *
     * @return The maximum angular velocity of the drivetrain.
     */
    @Override
    protected double getMaxAngularVelocity() {
        return getMaxVelocity() / (TRACK_WIDTH_METERS / 2);
    }

    /**
     * Gets the maximum acceleration of the drivetrain.
     *
     * @return The maximum acceleration of the drivetrain.
     */
    @Override
    protected double getMaxAcceleration() {
        return MAX_ACCELERATION;
    }

    /**
     * Gets the maximum angular acceleration of the drivetrain.
     *
     * @return The maximum angular acceleration of the drivetrain.
     */
    @Override
    protected double getMaxAngularAcceleration() {
        return getMaxAcceleration() / (TRACK_WIDTH_METERS / 2);
    }

    /**
     * Get the supplier for the current robot pose
     *
     * @return The supplier for the current robot pose
     */
    @Override
    public Supplier<Pose2d> getRobotPoseSupplier() {
        return this::getPose;
    }

    /**
     * Get the consumer for the vision measurements, this is used by the vision subsystem to pass measurements to the
     * drive subsystem
     *
     * @return The consumer for the vision measurements
     */
    @Override
    public Consumer<VisionMeasurement> getVisionMeasurementConsumer() {
        return (measurement) -> {
            poseEstimator.addVisionMeasurement(measurement.robotPose(), measurement.timestamp(), measurement.stdDevs());
        };
    }

    /**
     * Get the supplier for the field relative speeds
     *
     * @return The supplier for the field relative speeds
     */
    @Override
    public Supplier<ChassisSpeeds> getFieldRelativeSpeedsSupplier() {
        return this::getFieldRelativeSpeeds;
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     *
     * @param xSupplier
     * @param ySupplier
     * @param omegaSupplier
     * @param deadband
     */
    @Override
    public Command joystickDrive(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, double deadband) {
        return Commands.run(
                () -> {
                    double xSpeed = MathUtil.applyDeadband(xSupplier.getAsDouble(), deadband);
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), deadband);

                    xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
                    omega = Math.copySign(omega * omega, omega);

                    this.setTargetSpeeds(
                            new ChassisSpeeds(xSpeed * this.getMaxVelocity(), 0, omega * this.getMaxAngularVelocity()));
                },
                this);
    }

    /**
     * Sets the robot to a specified pose.
     *
     * @param poseSupplier The pose to reset to.
     */
    @Override
    public Command setRobotPose(Supplier<Pose2d> poseSupplier) {
        return Commands.runOnce(() ->
                poseEstimator.resetPosition(robotAngle, leftInputs.position, rightInputs.position, poseSupplier.get()));
    }
}
