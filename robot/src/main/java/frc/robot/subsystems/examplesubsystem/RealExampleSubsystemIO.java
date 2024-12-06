/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.examplesubsystem;

import static frc.robot.constants.ExampleSubsystemConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

public class RealExampleSubsystemIO implements ExampleSubsystemIO {
    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    public RealExampleSubsystemIO() {
        motor = new SparkFlex(CAN_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();

        motor.configure(
                MOTOR_CONFIG, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(ExampleSubsystemInputs inputs) {
        inputs.motorPosition = encoder.getPosition();
        inputs.motorVelocity = encoder.getVelocity();
        inputs.motorAppliedVoltage = motor.getAppliedOutput() * 12;
        inputs.motorCurrentDraw = motor.getOutputCurrent();
        inputs.motorTemperature = motor.getMotorTemperature();
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity) {
        // Fancy PID here or just motor.set();
    }

    /** {@inheritDoc} */
    @Override
    public void setPosition(double position) {
        // Fancy PID control here
    }
}
