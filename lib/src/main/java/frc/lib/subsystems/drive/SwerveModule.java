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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import frc.lib.math.kinematics.NthOrderSwerveModuleState;
import org.littletonrobotics.junction.Logger;

/** Class to represent an individual swerve module. */
public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    private final int id;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    /**
     * Creates a new swerve module.
     *
     * @param io The IO for the swerve module.
     */
    public SwerveModule(SwerveModuleIO io, int id) {
        this.io = io;
        this.id = id;
    }

    /** This method should be called every cycle */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module" + id, inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositions[i];
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    /**
     * Sets the target state for the swerve module. It does not set the target state for the rotation if there is no
     * desired speed.
     *
     * @param state The target state for the swerve module.
     */
    public void setTargetState(NthOrderSwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > 1.0e-5) {
            io.setTurnState(state.angle.getRadians(), state.angularVelocity.in(Units.RadiansPerSecond));
        }
        state.cosineScale(getTurnAngle()); // Slow down the module if the error is too large
        io.setDriveState(state.speedMetersPerSecond, state.acceleration.in(Units.MetersPerSecondPerSecond));
    }

    /**
     * Gets the current state of the swerve module.
     *
     * @return The current state of the swerve module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurnAngle());
    }

    /**
     * Gets the position of the swerve module.
     *
     * @return The position of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), getTurnAngle());
    }

    /**
     * Get all the position of the swerve module retrieved by the odometry thread this cycle
     *
     * @return Array of the swerve module positions
     */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    /**
     * Gets the absolute rotation angle of the module
     *
     * @return the absolute rotation angle from inputs
     */
    public Rotation2d getTurnAngle() {
        return inputs.turnAbsoluteRotation;
    }

    /**
     * Gets the drive distance of the module
     *
     * @return the drive position from inputs
     */
    public double getDriveDistance() {
        return inputs.drivePosition;
    }

    /**
     * Gets the drive velocity of the module
     *
     * @return the drive velocity from inputs
     */
    public double getDriveVelocity() {
        return inputs.driveVelocity;
    }
}
