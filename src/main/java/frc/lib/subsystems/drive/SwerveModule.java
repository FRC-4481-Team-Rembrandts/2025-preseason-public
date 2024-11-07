package frc.lib.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/**
 * Class to represent an individual swerve module.
 */
public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();
    private final int id;

    /**
     * Creates a new swerve module.
     *
     * @param io The IO for the swerve module.
     */
    public SwerveModule(SwerveModuleIO io, int id) {
        this.io = io;
        this.id = id;
    }

    /**
     * This method should be called every cycle
     */
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module" + id, inputs);
    }

    /**
     * Sets the target state for the swerve module.
     * It does not set the target state for the rotation if there is no desired speed.
     *
     * @param state The target state for the swerve module.
     */
    public void setTargetState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > 1.0e-5){
            io.setTurnRotation(state.angle.getRadians());
        }
        io.setDriveVelocity(state.speedMetersPerSecond);
    }

    /**
     * Gets the current state of the swerve module.
     *
     * @return The current state of the swerve module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getTurnAngle());
    }

    /**
     * Gets the position of the swerve module.
     *
     * @return The position of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), getTurnAngle());
    }

    /**
     * Gets the absolute rotation angle of the module
     *
     * @return the absolute rotation angle from inputs
     */
    public Rotation2d getTurnAngle(){
        return inputs.turnAbsoluteRotation;
    }

    /**
     * Gets the drive distance of the module
     *
     * @return the drive position from inputs
     */
    public double getDriveDistance(){
        return inputs.drivePosition;
    }

    /**
     * Gets the drive velocity of the module
     *
     * @return the drive velocity from inputs
     */
    public double getDriveVelocity(){
        return inputs.driveVelocity;
    }

}
