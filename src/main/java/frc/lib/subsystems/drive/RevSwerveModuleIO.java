package frc.lib.subsystems.drive;

import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.control.PIDFGains;

/**
 * Abstract class to define common functionality for swerve modules using REV Robotics electronics.
 * This class should be extended by a class that implements the specific motor controller used for the drive motor.
 *
 * <p> The turn motor is assumed to be controlled by a Spark MAX motor controller.
 * The absolute encoder on the turn motor is assumed to be a REV Through Bore encoder
 * connected directly to the turn motor controller.
 */
public abstract class RevSwerveModuleIO implements SwerveModuleIO {

    private final CANSparkBase driveMotor;
    private final CANSparkBase turnMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkAbsoluteEncoder turnAbsoluteEncoder;

    private final SimpleMotorFeedforward driveFeedForward;
    private final SimpleMotorFeedforward turnFeedForward;

    /**
     * Creates a new {@code RevSwerveModule}.
     *
     * @param driveId           The CAN id of the drive motor.
     * @param turnId            The CAN id of the turn motor.
     * @param drivePID          PIDF constants for the drive motor.
     * @param turnPID           PIDF constants for the turn motor.
     * @param gearReduction     Reduction between the motor shaft and the output shaft.
     * @param wheelDiameter     Diameter of the wheel in the module.
     * @param driveInversion    Whether the drive motor is inverted.
     * @param currentLimitTurn  The current limit for the turn motor.
     * @param currentLimitDrive The current limit for the drive motor.
     * @param idleModeDrive     The idle mode of the drive motor.
     * @param idleModeTurn      The idle mode of the drive motor.
     */
    public RevSwerveModuleIO(
            int driveId,
            int turnId,
            PIDFGains drivePID,
            PIDFGains turnPID,
            double gearReduction,
            double wheelDiameter,
            boolean driveInversion,
            int currentLimitDrive,
            int currentLimitTurn,
            CANSparkBase.IdleMode idleModeDrive,
            CANSparkBase.IdleMode idleModeTurn
    ) {
        driveMotor = setDriveMotor(driveId);
        turnMotor = setTurnMotor(turnId);

        // Ensure the same initial state on every startup.
        driveMotor.restoreFactoryDefaults();
        turnMotor.restoreFactoryDefaults();

        // Encoder and PID controller setup
        driveEncoder = driveMotor.getEncoder();
        turnAbsoluteEncoder = turnMotor.getAbsoluteEncoder();
        SparkPIDController driveController = driveMotor.getPIDController();
        SparkPIDController turnController = turnMotor.getPIDController();
        driveController.setFeedbackDevice(driveEncoder);
        turnController.setFeedbackDevice(turnAbsoluteEncoder);

        // Conversion factors
        final double DRIVE_POSITION_CONVERSION_FACTOR = (Math.PI * wheelDiameter) / gearReduction;  // Rotations to rad
        final double DRIVE_VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR / 60;  // RPM to rad/s
        final double TURN_POSITION_CONVERSION_FACTOR = 2.0 * Math.PI;   // Rotations to rad
        final double TURN_VELOCITY_CONVERSION_FACTOR = 2.0 * Math.PI;   // Rot/s to rad/s

        driveEncoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR);
        driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_CONVERSION_FACTOR);
        turnAbsoluteEncoder.setPositionConversionFactor(TURN_POSITION_CONVERSION_FACTOR);
        turnAbsoluteEncoder.setVelocityConversionFactor(TURN_VELOCITY_CONVERSION_FACTOR);

        turnMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5, 5);
        turnAbsoluteEncoder.setInverted(true);

        driveMotor.setIdleMode(idleModeDrive);
        turnMotor.setIdleMode(idleModeTurn);

        // PID wrapping for turn motors
        turnController.setPositionPIDWrappingEnabled(true);
        turnController.setPositionPIDWrappingMinInput(0);
        turnController.setPositionPIDWrappingMaxInput(TURN_POSITION_CONVERSION_FACTOR);

        // Drive PID
        driveController.setP(drivePID.pid().kP());
        driveController.setI(drivePID.pid().kI());
        driveController.setD(drivePID.pid().kD());

        // Turn PID
        turnController.setP(turnPID.pid().kP());
        turnController.setI(turnPID.pid().kI());
        turnController.setD(turnPID.pid().kD());

        // Feedforward
        driveFeedForward = new SimpleMotorFeedforward(
                drivePID.ff().kS(),
                drivePID.ff().kV(),
                drivePID.ff().kA()
        );
        turnFeedForward = new SimpleMotorFeedforward(
                turnPID.ff().kS(),
                turnPID.ff().kV(),
                turnPID.ff().kA()
        );

        driveMotor.setInverted(driveInversion);

        driveMotor.setSmartCurrentLimit(currentLimitDrive);
        turnMotor.setSmartCurrentLimit(currentLimitTurn);

        // Save config to the motor controllers
        driveMotor.burnFlash();
        turnMotor.burnFlash();
    }

    /**
     * {@inheritDoc}
     */
    public void updateInputs(SwerveModuleInputs inputs) {
        inputs.drivePosition = driveEncoder.getPosition();
        inputs.driveVelocity = driveEncoder.getVelocity();
        inputs.driveAppliedVoltage = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveCurrent = driveMotor.getOutputCurrent();
        inputs.driveTemperature = driveMotor.getMotorTemperature();

        inputs.turnAbsoluteRotation = new Rotation2d(turnAbsoluteEncoder.getPosition());
        inputs.turnAngularVelocity = turnAbsoluteEncoder.getVelocity();
        inputs.turnAppliedVoltage = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnCurrent = turnMotor.getOutputCurrent();
        inputs.turnTemperature = turnMotor.getMotorTemperature();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setDriveVelocity(double velocity) {
        driveMotor.getPIDController().setReference(
                velocity,
                CANSparkBase.ControlType.kVelocity,
                0,
                driveFeedForward.calculate(velocity),
                SparkPIDController.ArbFFUnits.kVoltage
        );
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setTurnRotation(double rotation) {
        turnMotor.getPIDController().setReference(
                rotation,
                CANSparkBase.ControlType.kPosition,
                0,
                turnFeedForward.calculate(0), //The desired velocity of the module is left at 0 for now, position does not matter for FF
                SparkPIDController.ArbFFUnits.kVoltage
        );

    }

    /**
     * {@inheritDoc}
     */
    public void setDriveBrakeMode(boolean enabled) {
        driveMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    /**
     * {@inheritDoc}
     */
    public void setTurnBrakeMode(boolean enabled) {
        turnMotor.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }

    /**
     * Sets the drive motor of the module.
     * The drive motor should be a concrete implementation of CANSparkBase.
     *
     * @param id The CAN id of the drive motor.
     * @return The drive motor.
     */
    protected abstract CANSparkBase setDriveMotor(int id);

    /**
     * Sets the turn motor of the module.
     * The turn motor should be a concrete implementation of CANSparkBase.
     *
     * @param id The CAN id of the turn motor.
     * @return The turn motor.
     */
    protected abstract CANSparkBase setTurnMotor(int id);
}