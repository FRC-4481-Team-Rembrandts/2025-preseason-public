package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import static frc.robot.Constants.Climber.CURRENT_LIMIT;

public class PhysicalClimberIO implements ClimberIO {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public PhysicalClimberIO(int canId, boolean isInverted) {
        motor = new CANSparkMax(canId, CANSparkLowLevel.MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motor.setSmartCurrentLimit(CURRENT_LIMIT);
        motor.setInverted(isInverted);
        motor.burnFlash();

        encoder = motor.getEncoder();
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();
        inputs.appliedVoltage = motor.getAppliedOutput();
        inputs.currentDraw = motor.getOutputCurrent();
        inputs.temperature = motor.getMotorTemperature();
    }

    /**
     * Sets speed of climber motor
     *
     * @param speed in percentage (-1 to 1)
     */
    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
