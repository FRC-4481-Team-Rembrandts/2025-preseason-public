/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.auto.AutoRoutine;
import frc.lib.path.ChoreoOpenLoopTrajectoryFollower;
import frc.lib.subsystems.drive.Drive;

public final class Autos {

    /** Example static factory for an autonomous command that follows a path. */
    @AutoRoutine(name = "Example path auto")
    public static Command examplePathAuto(Drive drive) {
        return Commands.sequence(drive.followTrajectory(new ChoreoOpenLoopTrajectoryFollower("test_path")));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
