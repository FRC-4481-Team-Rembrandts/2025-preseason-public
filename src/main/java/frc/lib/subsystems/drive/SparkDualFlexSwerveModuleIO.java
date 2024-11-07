package frc.lib.subsystems.drive;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import frc.lib.control.PIDFGains;

/**
 * Implementation of a REV swerve module using a SPARK MAX controller for the drive motor.
 */
public class SparkDualFlexSwerveModuleIO extends RevSwerveModuleIO {

    /**
     * Creates a new REV swerve module using a SPARK MAX controller for the drive motor.
     *
     * @param driveId The CAN id of the drive motor.
     * @param turnId The CAN id of the turn motor.
     * @param drivePID PIDF constants for the drive motor.
     * @param turnPID PIDF constants for the turn motor.
     * @param gearReduction Reduction between the motor shaft and the output shaft.
     * @param wheelDiameter Diameter of the wheel in the module.
     * @param driveInversion Whether the drive motor is inverted.
     * @param currentLimitTurn The current limit for the turn motor.
     * @param currentLimitDrive The current limit for the drive motor.
     * @param idleModeDrive The idle mode of the drive motor.
     * @param idleModeTurn The idle mode of the drive motor.
     */
    public SparkDualFlexSwerveModuleIO(int driveId,
                                       int turnId,
                                       PIDFGains drivePID,
                                       PIDFGains turnPID,
                                       double gearReduction,
                                       double wheelDiameter,
                                       boolean driveInversion,
                                       int currentLimitTurn,
                                       int currentLimitDrive,
                                       CANSparkBase.IdleMode idleModeDrive,
                                       CANSparkBase.IdleMode idleModeTurn) {
        super(driveId, turnId, drivePID, turnPID, gearReduction, wheelDiameter, driveInversion, currentLimitTurn, currentLimitDrive, idleModeDrive, idleModeTurn);
    }

    /**
     * Sets the drive motor for the module.
     * @param id The CAN id of the drive motor.
     * @return The drive motor.
     */
    @Override
    protected CANSparkBase setDriveMotor(int id) {
        return new CANSparkFlex(id, CANSparkBase.MotorType.kBrushless);
    }

    @Override
    protected CANSparkBase setTurnMotor(int id) {
        return new CANSparkFlex(id, CANSparkBase.MotorType.kBrushless);
    }
}
