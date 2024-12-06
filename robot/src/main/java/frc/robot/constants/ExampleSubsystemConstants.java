/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.constants;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ExampleSubsystemConstants {

    public static final int CAN_ID = 52;

    public static final int CURRENT_LIMIT = 30;
    public static final double DRIVE_ENCODER_POSITION_FACTOR = 3.0;
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = 1.0;

    public static final boolean INVERTED = false;

    public static final double POSITION_MARGIN = 5;

    // Configs
    public static final SparkMaxConfig MOTOR_CONFIG = new SparkMaxConfig();

    static {
        MOTOR_CONFIG
                .encoder
                .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        MOTOR_CONFIG
                .inverted(INVERTED)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIMIT)
                .voltageCompensation(12.0);
        MOTOR_CONFIG
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
    }
}
