package frc.lib.auto;

import edu.wpi.first.wpilibj.DriverStation;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.Target;

/**
 * Indicates that a method is an autonomous routine that can be selected from the dashboard.
 */
@Retention(java.lang.annotation.RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface AutoRoutine {
    DriverStation.Alliance[] alliance() default {DriverStation.Alliance.Red, DriverStation.Alliance.Blue};
    String name();
}
