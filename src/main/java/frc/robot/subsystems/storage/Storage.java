package frc.robot.subsystems.storage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Storage.*;

public class Storage extends SubsystemBase {
    private final StorageIO io;

    private StorageInputsAutoLogged inputs = new StorageInputsAutoLogged();

    /** Creates a new Input */
    public Storage(StorageIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Storage", inputs);
    }

    /**
     * Creates a command that enables the storage.
     *
     * @return a command that sets the speed of the storage.
     */
    public Command enableStorageFastCommand() {
        return setSpeedCommand(STORAGE_SPEED_FULL, 0);
    }

    /**
     * Creates a command that enables the storage.
     *
     * @return a command that sets the speed of the storage.
     */
    public Command enableStorageSlowCommand() {
        return setSpeedCommand(STORAGE_SPEED_SLOW, 0);
    }


    /**
     * Creates a command that enables the storage.
     *
     * @return a command that sets the speed of the storage.
     */
    public Command enableReverseStorageCommand() {
        return setSpeedCommand(-STORAGE_SPEED_FULL, -FEEDER_SPEED);
    }

    public Command enableFeedingStorageCommand(){
        return setSpeedCommand(STORAGE_SPEED_FULL, FEEDER_SPEED);
    }

    /**
     * Creates a command that enables the storage.
     *
     * @return a command that sets the speed of the storage.
     */
    public Command disableStorageCommand() {
        return setSpeedCommand(0, 0);
    }

    private Command setSpeedCommand(double storageSpeed, double feederSpeed) {
        return Commands.run(
                () -> {
                    io.setStorageRollerSpeed(storageSpeed);
                    io.setFeederRollerSpeed(feederSpeed);
                }, this);
    }

    public Trigger noteEnteredStorage(){
        return new Trigger(() -> inputs.lowSensor).debounce(0.03);
    }

    public Trigger noteFullyInStorage(){
        return new Trigger(() -> inputs.upperSensor).debounce(0.03);
    }
}
