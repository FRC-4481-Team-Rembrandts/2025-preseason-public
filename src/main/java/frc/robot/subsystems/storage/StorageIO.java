package frc.robot.subsystems.storage;

import org.littletonrobotics.junction.AutoLog;

public interface StorageIO {
    @AutoLog
    class StorageInputs {
        public double storageRollerPosition;
        public double storageRollerVelocity;
        public double storageRollerAppliedVoltage;
        public double storageRollerCurrentDraw;
        public double storageRollerTemperature;

        public double feederRollerPosition;
        public double feederRollerVelocity;
        public double feederRollerAppliedVoltage;
        public double feederRollerCurrentDraw;
        public double feederRollerTemperature;

        public boolean lowSensor;
        public boolean upperSensor;
    }

    /**
     * Updates the inputs of the subsystem.
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(StorageInputs inputs){};

    /**
     * Sets speed of front roller motor
     *
     * @param speed in percentage (-1 to 1)
     */
    default void setStorageRollerSpeed(double speed) {};

    /**
     * Sets speed of center roller motor
     *
     * @param speed in percentage (-1 to 1)
     */
    default void setFeederRollerSpeed(double speed) {};
}


