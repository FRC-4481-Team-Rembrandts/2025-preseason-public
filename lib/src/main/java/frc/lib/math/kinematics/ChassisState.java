/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.math.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This class is an enhanced version of ChassisSpeeds that also includes the linear and angular acceleration of the
 * chassis.
 */
public class ChassisState extends ChassisSpeeds {
    /** The linear acceleration of the chassis along x-axis in m/s^2(Fwd is +). */
    public double ax;
    /** The linear acceleration of the chassis along y-axis in m/s^2 (Left is +). */
    public double ay;
    /** The angular acceleration of the chassis in rad/s^2 (CCW is +). */
    public double alpha;

    /**
     * Constructs a ChassisState object.
     *
     * @param vxMetersPerSecond Forward velocity
     * @param vyMetersPerSecond Sideways velocity
     * @param omegaRadiansPerSecond Angular velocity
     * @param ax Forward acceleration in m/s^2
     * @param ay Sideways acceleration in m/s^2
     * @param alpha Angular acceleration in rad/s^2
     */
    public ChassisState(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            double ax,
            double ay,
            double alpha) {
        super(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        this.ax = ax;
        this.ay = ay;
        this.alpha = alpha;
    }

    /**
     * Constructs a ChassisState object with 0 linear and angular acceleration.
     *
     * @param vxMetersPerSecond Forward velocity
     * @param vyMetersPerSecond Sideways velocity
     * @param omegaRadiansPerSecond Angular velocity
     */
    public ChassisState(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        this(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, 0, 0, 0);
    }

    /**
     * Constructs a ChassisState object from a ChassisSpeeds object with additional acceleration information.
     *
     * @param chassisSpeeds ChassisSpeeds object containing the velocities of the drivetrain
     * @param ax Forward acceleration in m/s^2
     * @param ay Sideways acceleration in m/s^2
     * @param alpha Angular acceleration in rad/s^2
     */
    public ChassisState(ChassisSpeeds chassisSpeeds, double ax, double ay, double alpha) {
        this(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond,
                ax,
                ay,
                alpha);
    }

    /** Construct a ChassisState with all values set to zero. */
    public ChassisState() {
        this(0, 0, 0, 0, 0, 0);
    }

    /**
     * Converts a user provided field-relative ChassisState to a robot-relative ChassisState.
     *
     * @param fieldRelativeState The ChassisState object representing the speeds in the field frame of reference.
     *     Positive x is away from your alliance wall. Positive y is to your left when standing behind the alliance
     *     wall.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero
     *     when it is facing directly away from your alliance station wall. Remember that this should be CCW positive.
     * @return ChassisState object representing the speeds and accelerations in the robot's frame of reference.
     */
    public static ChassisState fromFieldRelativeState(ChassisState fieldRelativeState, Rotation2d robotAngle) {
        return fromFieldRelativeState(
                fieldRelativeState.vxMetersPerSecond,
                fieldRelativeState.vyMetersPerSecond,
                fieldRelativeState.omegaRadiansPerSecond,
                fieldRelativeState.ax,
                fieldRelativeState.ay,
                fieldRelativeState.alpha,
                robotAngle);
    }

    /**
     * Converts a user provided field-relative ChassisState to a robot-relative ChassisState. Positive x is away from
     * your alliance wall. Positive y is to your left when standing behind the alliance wall.
     *
     * @param vxMetersPerSecond Forward velocity
     * @param vyMetersPerSecond Sideways velocity
     * @param omegaRadiansPerSecond Angular velocity
     * @param ax Forward acceleration in m/s^2
     * @param ay Sideways acceleration in m/s^2
     * @param alpha Angular acceleration in rad/s^2
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero
     *     when it is facing directly away from your alliance station wall. Remember that this should be CCW positive.
     * @return ChassisState object representing the speeds and accelerations in the robot's frame of reference.
     */
    public static ChassisState fromFieldRelativeState(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            double ax,
            double ay,
            double alpha,
            Rotation2d robotAngle) {
        Translation2d rotatedVelocity =
                (new Translation2d(vxMetersPerSecond, vyMetersPerSecond)).rotateBy(robotAngle.unaryMinus());
        Translation2d rotatedAcceleration = (new Translation2d(ax, ay)).rotateBy(robotAngle.unaryMinus());
        return new ChassisState(
                rotatedVelocity.getX(),
                rotatedVelocity.getY(),
                omegaRadiansPerSecond,
                rotatedAcceleration.getX(),
                rotatedAcceleration.getY(),
                alpha);
    }

    /**
     * Converts a user provided robot-relative ChassisState to a field-relative ChassisState.
     *
     * @param robotRelativeState The ChassisState object representing the speeds in the robot frame of reference.
     *     Positive x is towards the robot's front. Positive y is towards the robot's left.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero
     *     when it is facing directly away from your alliance station wall. Remember that this should be CCW positive.
     * @return ChassisState object representing the speeds and accelerations in the field's frame of reference.
     */
    public static ChassisState fromRobotRelativeState(ChassisState robotRelativeState, Rotation2d robotAngle) {
        return fromRobotRelativeState(
                robotRelativeState.vxMetersPerSecond,
                robotRelativeState.vyMetersPerSecond,
                robotRelativeState.omegaRadiansPerSecond,
                robotRelativeState.ax,
                robotRelativeState.ay,
                robotRelativeState.alpha,
                robotAngle);
    }

    /**
     * Converts a user provided robot-relative ChassisState to a field-relative ChassisState. Positive x is towards the
     * robot's front. Positive y is towards the robot's left.
     *
     * @param vxMetersPerSecond Forward velocity
     * @param vyMetersPerSecond Sideways velocity
     * @param omegaRadiansPerSecond Angular velocity
     * @param axMetersPerSecondSquared Forward acceleration
     * @param ayMetersPerSecondSquared Sideways acceleration
     * @param alphaRadiansPerSecondSquared Angular acceleration
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is considered to be zero
     *     when it is facing directly away from your alliance station wall. Remember that this should be CCW positive.
     * @return ChassisState object representing the speeds and accelerations in the field's frame of reference.
     */
    public static ChassisState fromRobotRelativeState(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            double axMetersPerSecondSquared,
            double ayMetersPerSecondSquared,
            double alphaRadiansPerSecondSquared,
            Rotation2d robotAngle) {
        Translation2d rotatedVelocity = (new Translation2d(vxMetersPerSecond, vyMetersPerSecond)).rotateBy(robotAngle);
        Translation2d rotatedAcceleration =
                (new Translation2d(axMetersPerSecondSquared, ayMetersPerSecondSquared)).rotateBy(robotAngle);
        return new ChassisState(
                rotatedVelocity.getX(),
                rotatedVelocity.getY(),
                omegaRadiansPerSecond,
                rotatedAcceleration.getX(),
                rotatedAcceleration.getY(),
                alphaRadiansPerSecondSquared);
    }

    /**
     * Add two ChassisStates and return the sum.
     *
     * @param other The ChassisState to add.
     * @return The sum of the two ChassisStates.
     */
    public ChassisState plus(ChassisState other) {
        return new ChassisState(super.plus(other), this.ax + other.ax, this.ay + other.ay, this.alpha + other.alpha);
    }

    /**
     * Subtract two ChassisStates and return the difference.
     *
     * @param other The ChassisState to subtract.
     * @return The difference of the two ChassisStates.
     */
    public ChassisState minus(ChassisState other) {
        return new ChassisState(super.minus(other), this.ax - other.ax, this.ay - other.ay, this.alpha - other.alpha);
    }

    /**
     * Multiply a ChassisState by a scalar and return the result.
     *
     * @param scalar The scalar to multiply by.
     * @return The ChassisState multiplied by the scalar.
     */
    public ChassisState times(double scalar) {
        return new ChassisState(super.times(scalar), this.ax * scalar, this.ay * scalar, this.alpha * scalar);
    }

    /**
     * Returns the inverse of the current ChassisState. This is equivalent to negating all components of the
     * ChassisState.
     *
     * @return The negated ChassisState.
     */
    public ChassisState unaryMinus() {
        return new ChassisState(super.unaryMinus(), -this.ax, -this.ay, -this.alpha);
    }

    /**
     * Divide a ChassisState by a scalar and return the result.
     *
     * @param scalar The scalar to divide by.
     * @return The ChassisState divided by the scalar.
     */
    public ChassisState div(double scalar) {
        return new ChassisState(super.div(scalar), this.ax / scalar, this.ay / scalar, this.alpha / scalar);
    }

    /**
     * Return a ChassisSpeeds object representing the velocities of this ChassisState.
     *
     * @return ChassisSpeeds object based on this ChassisState.
     */
    public ChassisSpeeds toChassisSpeeds() {
        return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }
}
