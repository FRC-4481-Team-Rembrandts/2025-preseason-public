/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.Units;
import frc.lib.control.FeedForwardGains;
import frc.lib.control.PIDGains;
import frc.lib.hardware.constants.GearRatios;
import frc.lib.hardware.constants.SwerveInversions;

public class DriveConstants {
    public static final double MAX_VELOCITY = 4.5;
    public static final double MAX_ACCELERATION = 3.0;
    public static final double ODOMETRY_FREQUENCY = 250.0;

    public static final double TRACK_WIDTH = 0.3606;
    public static final double WHEEL_BASE = 0.3606;
    public static final double DRIVEBASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

    // CAN IDs
    public static final int CAN_ID_IMU = 9;

    public static final int CAN_ID_FRONT_LEFT_DRIVE = 11;
    public static final int CAN_ID_FRONT_RIGHT_DRIVE = 13;
    public static final int CAN_ID_BACK_LEFT_DRIVE = 15;
    public static final int CAN_ID_BACK_RIGHT_DRIVE = 17;

    public static final int CAN_ID_FRONT_LEFT_TURN = 12;
    public static final int CAN_ID_FRONT_RIGHT_TURN = 14;
    public static final int CAN_ID_BACK_LEFT_TURN = 16;
    public static final int CAN_ID_BACK_RIGHT_TURN = 18;

    // Motor constants
    public static final boolean FRONT_LEFT_INVERTED = true;
    public static final boolean FRONT_RIGHT_INVERTED = false;
    public static final boolean BACK_LEFT_INVERTED = true;
    public static final boolean BACK_RIGHT_INVERTED = false;
    public static final double WHEEL_DIAMETER = Units.Meter.convertFrom(3, Units.Inch);
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 38;
    public static final double DRIVE_MOTOR_REDUCTION = GearRatios.RevRobotics.MaxSwerve.REDUCTION_HIGH_SPEED;

    public static final int TURN_MOTOR_CURRENT_LIMIT = 20;
    public static final double TURN_MOTOR_REDUCTION = 2.89 * 3.61 / 14 * 62;

    // Encoder constants
    public static final double DRIVE_ENCODER_POSITION_FACTOR = Math.PI / DRIVE_MOTOR_REDUCTION * WHEEL_DIAMETER;
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = Math.PI / 60.0 / DRIVE_MOTOR_REDUCTION * WHEEL_DIAMETER;

    // Turn encoder configuration
    public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
    public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Closed-loop constants
    public static final PIDGains DRIVE_PID = new PIDGains(0, 0, 0);
    public static final FeedForwardGains DRIVE_FF = new FeedForwardGains(0.12249, 0.09722, 0);
    public static final PIDGains TURN_PID = new PIDGains(2, 0, 0);
    public static final FeedForwardGains TURN_FF = new FeedForwardGains(0.2, 0.6, 0);

    // Configs
    public static SparkFlexConfig DRIVE_CONFIG_BASE = new SparkFlexConfig();
    public static SparkFlexConfig DRIVE_CONFIG_FRONT_LEFT = new SparkFlexConfig();
    public static SparkFlexConfig DRIVE_CONFIG_FRONT_RIGHT = new SparkFlexConfig();
    public static SparkFlexConfig DRIVE_CONFIG_BACK_LEFT = new SparkFlexConfig();
    public static SparkFlexConfig DRIVE_CONFIG_BACK_RIGHT = new SparkFlexConfig();

    static {
        DRIVE_CONFIG_BASE
                .encoder
                .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        DRIVE_CONFIG_BASE
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
                .pidf(DRIVE_PID.kP(), DRIVE_PID.kI(), DRIVE_PID.kD(), 0.0);
        DRIVE_CONFIG_BASE
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT)
                .voltageCompensation(12.0);
        DRIVE_CONFIG_BASE
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        // Apply the separate motor inversion settings
        DRIVE_CONFIG_FRONT_LEFT.apply(DRIVE_CONFIG_BASE);
        DRIVE_CONFIG_FRONT_LEFT.inverted(SwerveInversions.RevRobotics.MaxSwerve.FRONT_LEFT_INVERTED);
        DRIVE_CONFIG_FRONT_RIGHT.apply(DRIVE_CONFIG_BASE);
        DRIVE_CONFIG_FRONT_RIGHT.inverted(SwerveInversions.RevRobotics.MaxSwerve.FRONT_RIGHT_INVERTED);
        DRIVE_CONFIG_BACK_LEFT.apply(DRIVE_CONFIG_BASE);
        DRIVE_CONFIG_BACK_LEFT.inverted(SwerveInversions.RevRobotics.MaxSwerve.BACK_LEFT_INVERTED);
        DRIVE_CONFIG_BACK_RIGHT.apply(DRIVE_CONFIG_BASE);
        DRIVE_CONFIG_BACK_RIGHT.inverted(SwerveInversions.RevRobotics.MaxSwerve.BACK_RIGHT_INVERTED);
    }

    public static final SparkFlexConfig TURN_CONFIG = new SparkFlexConfig();

    static {
        TURN_CONFIG
                .absoluteEncoder
                .inverted(SwerveInversions.RevRobotics.MaxSwerve.REV_THROUGH_BORE_ENCODER_INVERTED)
                .positionConversionFactor(TURN_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(TURN_ENCODER_VELOCITY_FACTOR)
                .averageDepth(2);
        TURN_CONFIG
                .closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI)
                .pidf(TURN_PID.kP(), TURN_PID.kI(), TURN_PID.kD(), 0.0);
        TURN_CONFIG
                .inverted(SwerveInversions.RevRobotics.MaxSwerve.TURN_MOTOR_INVERTED)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(TURN_MOTOR_CURRENT_LIMIT)
                .voltageCompensation(12.0);
        TURN_CONFIG
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
    }
}
