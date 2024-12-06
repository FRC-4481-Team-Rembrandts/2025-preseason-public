/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 3 as published by the Free Software Foundation or
 * available in the root directory of this project.
 */
package frc.lib.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** A utility class for selecting autonomous routines from the dashboard. */
public class AutoSelector {
    /** A record representing an autonomous routine option. */
    record AutoOption(String name, Command autoRoutine) {}

    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

    private final List<SubsystemBase> subsystemList;

    /**
     * Creates a new AutoSelector.
     *
     * @param autoRoutineClass The class containing the autonomous routines.
     * @param list The list of subsystems which are used to pass to the autonomous routines.
     */
    public AutoSelector(Class autoRoutineClass, List<SubsystemBase> list) {
        subsystemList = list;

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

        for (AutoOption option : fetchAutoOptions(autoRoutineClass)) {
            autoChooser.addOption(option.name(), option.autoRoutine());
        }
    }

    /**
     * Gets the selected autonomous routine.
     *
     * @return The selected autonomous routine.
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Fetches the autonomous routine options from the given class.
     *
     * @param autoRoutineClass The class containing the autonomous routines.
     * @return The autonomous routine options.
     */
    private AutoOption[] fetchAutoOptions(Class autoRoutineClass) {
        List<AutoOption> options = new ArrayList<>();

        for (Method method : autoRoutineClass.getDeclaredMethods()) {
            if (method.isAnnotationPresent(AutoRoutine.class) && !method.isAnnotationPresent(Disabled.class)) {
                try {
                    options.add(new AutoOption(
                            method.getAnnotation(AutoRoutine.class).name(),
                            (Command) method.invoke(null, selectSubsystems(subsystemList, method))));
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }

        return options.toArray(new AutoOption[0]);
    }

    /**
     * Select the subsystems to pass as argument to the autonomous routine
     *
     * @param subsystemList List of all the subsystems
     * @param method The method of te autonomous routine
     */
    private static Object[] selectSubsystems(List<SubsystemBase> subsystemList, Method method) {
        // Get the parameter types of the method
        Class<?>[] parameterTypes = method.getParameterTypes();

        // Create an array to hold the arguments for the method
        Object[] argsForMethod = new Object[parameterTypes.length];

        // Create temporary list of the subsystems that we can edit
        List<SubsystemBase> tempSubsystemList = new ArrayList<>(subsystemList);

        // Loop through the method's parameter types
        for (int i = 0; i < parameterTypes.length; i++) {
            // Find the corresponding object in the list that matches the parameter type
            for (Object obj : tempSubsystemList) {
                if (parameterTypes[i].isInstance(obj)) {
                    argsForMethod[i] = obj;
                    tempSubsystemList.remove(obj); // Remove object from the list to make sure same object is not
                    // passed twice
                    break;
                }
            }
        }

        // Check if all parameters have been filled
        for (Object obj : argsForMethod) {
            if (obj == null) {
                throw new IllegalArgumentException(
                        "Autonomous routine ["
                                + method.getName()
                                + "] has missing parameters, "
                                + "check that all subsystems are added to the subsystem list and that the auto routine only has subsystems as parameters");
            }
        }

        return argsForMethod;
    }
}
