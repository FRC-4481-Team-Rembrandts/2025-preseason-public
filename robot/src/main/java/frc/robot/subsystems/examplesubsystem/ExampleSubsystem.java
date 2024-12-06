/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.examplesubsystem;

import static frc.robot.constants.ExampleSubsystemConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class ExampleSubsystem extends SubsystemBase {
    private final ExampleSubsystemIO io;
    private final ExampleSubsystemInputsAutoLogged inputs = new ExampleSubsystemInputsAutoLogged();
    private double positionSetpoint;

    /** Creates a new ExampleSubsystem. */
    public ExampleSubsystem(ExampleSubsystemIO io) {
        this.io = io;
    }

    /** Example of a trigger */
    public Trigger onPositionSetpoint() {
        return new Trigger(() -> {
            return Math.abs(inputs.motorPosition - positionSetpoint) < POSITION_MARGIN;
        });
    }

    /**
     * Example command factory method (method that returns a command) to go to a certain setpoint. This command makes
     * use of the private command that takes a position as input.
     *
     * @return Command to go to the high setpoint of the subsystem.
     */
    public Command goToHighSetpoint() {
        return runToTargetPosition(110);
    }

    /**
     * Example command factory method (method that returns a command) to go to a certain setpoint. This command makes
     * use of the private command that takes a position as input.
     *
     * @return Command to go to the low setpoint of the subsystem.
     */
    public Command goToLowSetpoint() {
        return runToTargetPosition(-30);
    }

    /**
     * Example command factory method (method that returns a command) to enable the subsystem at a certain velocity.
     *
     * @return Command to turn on the motor of the subsystem.
     */
    public Command enableForward() {
        return Commands.run(
                () -> {
                    // Enable the subsystem
                    io.setVelocity(3000);
                },
                this);
    }

    /**
     * Example command factory method (method that returns a command) to enable the subsystem at a certain velocity.
     *
     * @return Command to turn on the motor of the subsystem.
     */
    public Command enableBackward() {
        return Commands.run(
                () -> {
                    // Enable the subsystem
                    io.setVelocity(-3000);
                },
                this);
    }

    /**
     * Example command factory method (method that returns a command) to disable the subsystem.
     *
     * @return Command to turn off the subsystem.
     */
    public Command disable() {
        return Commands.run(
                () -> {
                    // Enable the subsystem
                    io.setVelocity(0);
                },
                this);
    }

    /**
     * Private command factory method to go to a certain position. Generally, commands with velocity or position inputs
     * should stay private to isolate the exact subsystem implementation from the rest of the code.
     *
     * @param position The target position
     * @return Command to go to the specified position.
     */
    private Command runToTargetPosition(double position) {
        return Commands.run(
                () -> {
                    // Set the position of the subsystem
                    positionSetpoint = position;
                    io.setPosition(position);
                },
                this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("ExampleSubsystem", inputs);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
