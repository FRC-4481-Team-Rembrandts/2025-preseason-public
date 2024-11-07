package frc.robot.subsystems.shamper;

import com.revrobotics.*;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class PhysicalShamperIO implements ShamperIO {

    private final CANSparkFlex topRoller;
    private final CANSparkFlex bottomRoller;
    private final CANSparkFlex kicker;
    private final CANSparkMax pivot;

    private SparkPIDController topPID;
    private SparkPIDController bottomPID;
    private SparkPIDController kickerPID;
    private SparkPIDController pivotPID;

    /**
     * Creates a new Shamper IO for a physical robot.
     */
    public PhysicalShamperIO() {
        // Top Roller
        topRoller = new CANSparkFlex(Constants.HardwareMap.TOP_SHOOTER_CAN_ID, CANSparkMax.MotorType.kBrushless);
        topRoller.restoreFactoryDefaults();
        topRoller.setSmartCurrentLimit(Constants.Shamper.CURRENT_LIMIT_SHOOTER);
        topRoller.setIdleMode(CANSparkBase.IdleMode.kCoast);
        topRoller.setInverted(true);

        topPID = topRoller.getPIDController();
        topPID.setP(Constants.Shamper.TOP_KP,0);
        topPID.setI(Constants.Shamper.TOP_KI,0);
        topPID.setIZone(Constants.Shamper.SHAMPER_KI_DEADZONE, 0);
        topPID.setIMaxAccum(Constants.Shamper.SHAMPER_KI_MAX_ACCUM, 0);

        topRoller.burnFlash();

        // Bottom Roller
        bottomRoller = new CANSparkFlex(Constants.HardwareMap.BOTTOM_SHOOTER_CAN_ID, CANSparkMax.MotorType.kBrushless);
        bottomRoller.restoreFactoryDefaults();
        bottomRoller.setSmartCurrentLimit(Constants.Shamper.CURRENT_LIMIT_SHOOTER);
        bottomRoller.setIdleMode(CANSparkBase.IdleMode.kCoast);
        bottomRoller.setInverted(true);

        bottomPID = bottomRoller.getPIDController();
        bottomPID.setP(Constants.Shamper.BOTTOM_KP,0);
        bottomPID.setI(Constants.Shamper.BOTTOM_KI,0);
        bottomPID.setIZone(Constants.Shamper.SHAMPER_KI_DEADZONE, 0);
        bottomPID.setIMaxAccum(Constants.Shamper.SHAMPER_KI_MAX_ACCUM, 0);

        bottomRoller.burnFlash();

        // Kicker
        kicker = new CANSparkFlex(Constants.HardwareMap.KICKER_SHOOTER_CAN_ID, CANSparkMax.MotorType.kBrushless);
        kicker.restoreFactoryDefaults();
        kicker.setSmartCurrentLimit(Constants.Shamper.CURRENT_LIMIT_SHOOTER);
        kicker.setIdleMode(CANSparkBase.IdleMode.kCoast);
        kicker.setInverted(true);

        RelativeEncoder kickerEncoder = kicker.getEncoder();
        kickerEncoder.setVelocityConversionFactor(Constants.Shamper.CONVERSION_FACTOR_KICKER);

        kickerPID = kicker.getPIDController();
        kickerPID.setP(Constants.Shamper.KICKER_KP,0);
        kickerPID.setI(Constants.Shamper.KICKER_KI,0);
        kickerPID.setIZone(Constants.Shamper.SHAMPER_KI_DEADZONE, 0);
        kickerPID.setIMaxAccum(Constants.Shamper.SHAMPER_KI_MAX_ACCUM, 0);

        kicker.burnFlash();

        // Pivot
        pivot = new CANSparkMax(Constants.HardwareMap.PIVOT_CAN_ID, CANSparkMax.MotorType.kBrushless);
        pivot.restoreFactoryDefaults();
        pivot.setInverted(true);
        pivot.setSmartCurrentLimit(Constants.Shamper.CURRENT_LIMIT_PIVOT);
        pivot.setIdleMode(CANSparkBase.IdleMode.kBrake);

        AbsoluteEncoder absoluteEncoder = pivot.getAbsoluteEncoder();
        absoluteEncoder.setInverted(true);
        absoluteEncoder.setPositionConversionFactor(360);

        pivotPID = pivot.getPIDController();
        pivotPID.setFeedbackDevice(absoluteEncoder);
        pivotPID.setP(Constants.Shamper.PIVOT_KP, 0);
        pivotPID.setP(0,1);


        pivot.burnFlash();
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    @Override
    public void updateInputs(ShamperInputs inputs) {
        inputs.topRollerVelocity = topRoller.getEncoder().getVelocity();
        inputs.topRollerAppliedOutput = topRoller.getAppliedOutput();
        inputs.topRollerCurrentDraw = topRoller.getOutputCurrent();
        inputs.topRollerTemperature = topRoller.getMotorTemperature();

        inputs.bottomRollerVelocity = bottomRoller.getEncoder().getVelocity();
        inputs.bottomRollerAppliedOutput = bottomRoller.getAppliedOutput();
        inputs.bottomRollerCurrentDraw = bottomRoller.getOutputCurrent();
        inputs.bottomRollerTemperature = bottomRoller.getMotorTemperature();

        inputs.kickerVelocity = kicker.getEncoder().getVelocity();
        inputs.kickerAppliedOutput = kicker.getAppliedOutput();
        inputs.kickerCurrentDraw = kicker.getOutputCurrent();
        inputs.kickerTemperature = kicker.getMotorTemperature();

        inputs.pivotAbsoluteEncoderPosition = pivot.getAbsoluteEncoder().getPosition();
        inputs.pivotRelativeEncoderPosition = pivot.getEncoder().getPosition();
        inputs.pivotVelocity = pivot.getEncoder().getVelocity();
        inputs.pivotAppliedOutput = pivot.getAppliedOutput();
        inputs.pivotCurrentDraw = pivot.getOutputCurrent();
        inputs.pivotTemperature = pivot.getMotorTemperature();

    }

    @Override
    public void setTopRollerSpeed(double speed) {
        Logger.recordOutput("Shamper/topRoller setpoint", speed);
        topPID.setReference(speed, CANSparkBase.ControlType.kVelocity, 0, speed * Constants.Shamper.TOP_FF, SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void setBottomRollerSpeed(double speed) {
        Logger.recordOutput("Shamper/bottomRoller setpoint", speed);
        bottomPID.setReference(speed, CANSparkBase.ControlType.kVelocity, 0, speed * Constants.Shamper.BOTTOM_FF, SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void setKickerSpeed(double speed) {
        Logger.recordOutput("Shamper/kicker setpoint", speed);
        kickerPID.setReference(speed, CANSparkBase.ControlType.kVelocity, 0, speed * Constants.Shamper.KICKER_FF, SparkPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void setPivotPosition(double position) {
        position = Math.min(Math.max(position, Constants.Shamper.PIVOT_MIN_POSITION), Constants.Shamper.PIVOT_MAX_POSITION);

        pivotPID.setReference(position, CANSparkBase.ControlType.kPosition, 0);
    }

}
