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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPMotor;

/** An implementation of a differential drivetrain IO that represents one side of a default XRP robot. */
public class XRPDifferentialDriveIO implements DifferentialDriveIO {
    private static final double GEAR_RATIO = (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
    private static final double COUNTS_PER_MOTOR_SHAFT_REV = 12.0;
    private static final double COUNTS_PER_REVOLUTION = COUNTS_PER_MOTOR_SHAFT_REV * GEAR_RATIO; // 585.0
    private static final double WHEEL_DIAMETER = 0.06; // 60 mm

    private static final double kP = 1;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kS = 1.9;
    private static final double kV = 0;
    private static final double kA = 0;

    private final Encoder encoder;
    private final XRPMotor motor;
    private final PIDController pidController = new PIDController(kP, kI, kD);
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);

    private double velocitySetpoint = 0;

    /**
     * Creates a new XRPDifferentialDriveIO based on custom parameters.
     *
     * @param motor the custom motor object to use.
     * @param encoder the custom encoder object to use.
     */
    public XRPDifferentialDriveIO(XRPMotor motor, Encoder encoder) {
        this.motor = motor;
        this.encoder = encoder;

        encoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER) / COUNTS_PER_REVOLUTION);
        encoder.reset();
    }

    /**
     * Creates a new XRPDifferentialDriveIO based on the default configuration of an XRP robot.
     *
     * @param side which side of the robot this object represents.
     */
    public XRPDifferentialDriveIO(ModuleSide side) {
        this(side.getMotor(), side.getEncoder());
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(DifferentialDriveInputs inputs) {
        inputs.position = encoder.getDistance();
        inputs.velocity = encoder.getRate();
        inputs.appliedVoltages = new double[] {updateAppliedVoltage()};
        inputs.currentsDrawn = new double[] {};
        inputs.temperatures = new double[] {};
    }

    /** {@inheritDoc} */
    @Override
    public void setDriveVelocity(double velocity) {
        velocitySetpoint = velocity;
    }

    /**
     * Updates the amount of voltage that needs to be applied to the motor and applies it.
     *
     * @return the amount of voltage that was supplied.
     */
    private double updateAppliedVoltage() {
        SmartDashboard.putNumber("Drive/left_velocity", encoder.getDistance());
        double pidVoltage = pidController.calculate(encoder.getRate(), velocitySetpoint);
        double ffVoltage = feedForward
                .calculate(Units.MetersPerSecond.of(velocitySetpoint))
                .in(Units.Volt);

        double setpointVoltage = pidVoltage + ffVoltage;

        motor.setVoltage(setpointVoltage);
        return setpointVoltage;
    }

    /** Enum to represent one of the sides of a standard XRP drivetrain. */
    public enum ModuleSide {
        LEFT(new XRPMotor(0), new Encoder(4, 5), false),
        RIGHT(new XRPMotor(1), new Encoder(6, 7), true);

        private final Encoder encoder;
        private final XRPMotor motor;

        ModuleSide(XRPMotor motor, Encoder encoder, boolean inverted) {
            this.motor = motor;
            this.encoder = encoder;

            motor.setInverted(inverted);
        }

        Encoder getEncoder() {
            return encoder;
        }

        XRPMotor getMotor() {
            return motor;
        }
    }
}
