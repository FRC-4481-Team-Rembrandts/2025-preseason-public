/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.subsystems.examplesubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import frc.lib.simulation.SimpleMotorSim;

public class SimulatedExampleSubsystemIO implements ExampleSubsystemIO {
    private final SimpleMotorSim motorSim;

    public SimulatedExampleSubsystemIO() {
        motorSim = new SimpleMotorSim(Units.Second.of(1.5), 1.0 / 360, 15, Units.RPM.of(6000));
        motorSim.createPivotMechanism2d("ExampleSubsystem/mech2d", 1, 1, Rotation2d.fromDegrees(20));
    }

    /** {@inheritDoc} */
    @Override
    public void updateInputs(ExampleSubsystemInputs inputs) {
        inputs.motorPosition = motorSim.getCurrentPosition();
        inputs.motorVelocity = motorSim.getCurrentVelocity();
    }

    /** {@inheritDoc} */
    @Override
    public void setVelocity(double velocity) {
        motorSim.setTargetVelocity(velocity);
    }

    /** {@inheritDoc} */
    @Override
    public void setPosition(double position) {
        motorSim.setTargetPosition(position);
    }
}
