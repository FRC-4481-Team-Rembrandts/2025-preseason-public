package frc.lib.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface to define common IMU input/output functionality.
 */
public interface IMUIO {
    @AutoLog
    class IMUInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocity = 0.0;
    }

    /**
     * Updates the system with new inputs from the IMU.
     * @param inputs  Inputs from the IMU.
     */
    default void updateInputs(IMUInputs inputs) {}
}