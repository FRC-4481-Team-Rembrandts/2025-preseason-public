package frc.lib.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface to define common swerve module input/output functionality.
 */
public interface SwerveModuleIO {

    @AutoLog
    class SwerveModuleInputs {
        public double drivePosition = 0;
        public double driveVelocity = 0;
        public double driveAppliedVoltage = 0;
        public double driveCurrent = 0;

        public Rotation2d turnAbsoluteRotation = new Rotation2d();
        public double turnAngularVelocity = 0;
        public double turnAppliedVoltage = 0;
        public double turnCurrent = 0;

        public double driveTemperature = 0;
        public double turnTemperature = 0;
    }

    /**
     * Updates the system with new inputs from the module.
     * @param inputs The new inputs.
     */
    default void updateInputs(SwerveModuleInputs inputs) {}

    /**
     * Sets the target drive velocity for the module.
     * @param velocity target drive velocity in m/s.
     */
    default void setDriveVelocity(double velocity) {}

    /**
     * Sets the target turn position for the module.
     * @param rotation target turn position in radians.
     */
    default void setTurnRotation(double rotation) {}
}
