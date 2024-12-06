/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.examplesubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ExampleSubsystemIO {

    @AutoLog
    class ExampleSubsystemInputs {
        public double motorPosition;
        public double motorVelocity;
        public double motorAppliedVoltage;
        public double motorCurrentDraw;
        public double motorTemperature;
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(ExampleSubsystemInputs inputs) {}
    ;

    /**
     * Sets speed of the motor
     *
     * @param velocity in rpm of the output
     */
    default void setVelocity(double velocity) {}
    ;

    /**
     * Sets target position of the motor
     *
     * @param position in degrees of the output
     */
    default void setPosition(double position) {}
    ;
}
