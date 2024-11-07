package frc.robot.subsystems.storage;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Constants.HardwareMap.*;
import static frc.robot.Constants.HardwareMap.UPPER_SENSOR_ID;
import static frc.robot.Constants.Storage.CURRENT_LIMIT;

public class PhysicalStorageIO implements StorageIO {

    private final CANSparkMax storageRollerMotor;
    private final CANSparkFlex feederRollerMotor;

    private final RelativeEncoder frontRollerEncoder;
    private final RelativeEncoder centerRollerEncoder;

    private final DigitalInput lowSensor;
    private final DigitalInput upperSensor;

    public PhysicalStorageIO() {
        // Front Roller definition
        storageRollerMotor = new CANSparkMax(STORAGE_ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        storageRollerMotor.restoreFactoryDefaults();
        storageRollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        storageRollerMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        storageRollerMotor.setInverted(true);
        storageRollerMotor.burnFlash();

        frontRollerEncoder = storageRollerMotor.getEncoder();

        // Center Roller definition
        feederRollerMotor = new CANSparkFlex(FEEDER_ROLLER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        feederRollerMotor.restoreFactoryDefaults();
        feederRollerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        feederRollerMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        feederRollerMotor.setInverted(false);
        feederRollerMotor.burnFlash();

        centerRollerEncoder = feederRollerMotor.getEncoder();

        lowSensor = new DigitalInput(LOWER_SENSOR_ID);
        upperSensor = new DigitalInput(UPPER_SENSOR_ID);
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    @Override
    public void updateInputs(StorageInputs inputs) {
        inputs.storageRollerPosition = frontRollerEncoder.getPosition();
        inputs.storageRollerVelocity = frontRollerEncoder.getVelocity();
        inputs.storageRollerAppliedVoltage = storageRollerMotor.getAppliedOutput();
        inputs.storageRollerCurrentDraw = storageRollerMotor.getOutputCurrent();
        inputs.storageRollerTemperature = storageRollerMotor.getMotorTemperature();

        inputs.feederRollerPosition = centerRollerEncoder.getPosition();
        inputs.feederRollerVelocity = centerRollerEncoder.getVelocity();
        inputs.feederRollerAppliedVoltage = feederRollerMotor.getAppliedOutput();
        inputs.feederRollerCurrentDraw = feederRollerMotor.getOutputCurrent();
        inputs.feederRollerTemperature = feederRollerMotor.getMotorTemperature();

        inputs.lowSensor = !lowSensor.get();
        inputs.upperSensor = !upperSensor.get();
    }

    /**
     * Sets speed of front roller motor
     *
     * @param speed in percentage (-1 to 1)
     */
    @Override
    public void setStorageRollerSpeed(double speed) {
        storageRollerMotor.set(speed);
    }

    /**
     * Sets speed of center roller motor
     *
     * @param speed in percentage (-1 to 1)
     */
    @Override
    public void setFeederRollerSpeed(double speed) { feederRollerMotor.set(speed); }


}
