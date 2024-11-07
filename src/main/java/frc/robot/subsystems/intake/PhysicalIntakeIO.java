package frc.robot.subsystems.intake;

import com.revrobotics.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants;

import static frc.robot.Constants.HardwareMap.INTAKE_CENTER_MOTOR_ID;
import static frc.robot.Constants.HardwareMap.INTAKE_FRONT_MOTOR_ID;
import static java.lang.Math.abs;

import static frc.robot.Constants.Intake.*;

public class PhysicalIntakeIO implements IntakeIO {

    private final CANSparkFlex frontRollerMotor;
    private final CANSparkFlex centerRollerMotor;

    private final RelativeEncoder frontRollerEncoder;
    private final RelativeEncoder centerRollerEncoder;

    SlewRateLimiter slewRateLimiter = new SlewRateLimiter(INTAKE_SLEW_RATE);

    public PhysicalIntakeIO() {
        // Front Roller definition
        frontRollerMotor = new CANSparkFlex(INTAKE_FRONT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        frontRollerMotor.restoreFactoryDefaults();
        frontRollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        frontRollerMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        frontRollerMotor.setInverted(false);
        frontRollerMotor.burnFlash();

        frontRollerEncoder = frontRollerMotor.getEncoder();

        // Center Roller definition
        centerRollerMotor = new CANSparkFlex(INTAKE_CENTER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
        centerRollerMotor.restoreFactoryDefaults();
        centerRollerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        centerRollerMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        centerRollerMotor.setInverted(false);
        centerRollerMotor.burnFlash();

        centerRollerEncoder = centerRollerMotor.getEncoder();
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.frontRollerPosition = frontRollerEncoder.getPosition();
        inputs.frontRollerVelocity = frontRollerEncoder.getVelocity();
        inputs.frontRollerAppliedVoltage = frontRollerMotor.getAppliedOutput();
        inputs.frontRollerCurrentDraw = frontRollerMotor.getOutputCurrent();
        inputs.frontRollerTemperature = frontRollerMotor.getMotorTemperature();

        inputs.centerRollerPosition = centerRollerEncoder.getPosition();
        inputs.centerRollerVelocity = centerRollerEncoder.getVelocity();
        inputs.centerRollerAppliedVoltage = centerRollerMotor.getAppliedOutput();
        inputs.centerRollerCurrentDraw = centerRollerMotor.getOutputCurrent();
        inputs.centerRollerTemperature = centerRollerMotor.getMotorTemperature();
    }

    /**
     * Sets speed of front roller motor
     *
     * @param speed in percentage (-1 to 1)
     */
    @Override
    public void setFrontRollerSpeed(double speed) {
        frontRollerMotor.set(speed);
    }

    /**
     * Sets speed of center roller motor
     *
     * @param speed in percentage (-1 to 1)
     */
    @Override
    public void setCenterRollerSpeed(double speed) { centerRollerMotor.set(speed); }

    /**
     * Sets speed of both intake rollers
     * @param speed in percentage (-1 to 1)
     */
    @Override
    public void setSpeed(double speed){
        double filteredSpeed = slewRateLimiter.calculate(speed);
        frontRollerMotor.set(filteredSpeed);
        centerRollerMotor.set(filteredSpeed);
    }

}
