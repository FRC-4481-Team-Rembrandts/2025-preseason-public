package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Intake.*;

public class Intake extends SubsystemBase {
    private final IntakeIO io;

    private IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    /** Creates a new Input */
    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    /**
     * Creates a command that enables the intake.
     *
     * @return a command that sets the speed of the intake.
     */
    public Command enableIntakeCommand() {
        return setSpeedCommand(INTAKE_SPEED_FULL);
    }

    public Command enableIntakeSlowCommand() {
        return setSpeedCommand(INTAKE_SPEED_SLOW);
    }

    /**
     * Creates a command that enables the intake.
     *
     * @return a command that sets the speed of the intake.
     */
    public Command enableReverseIntakeCommand() {
        return setSpeedCommand(-INTAKE_SPEED_FULL);
    }

    /**
     * Creates a command that enables the intake.
     *
     * @return a command that sets the speed of the intake.
     */
    public Command disableIntakeCommand() {
        return setSpeedCommand(0.0);
    }

    private Command setSpeedCommand(double speed) {
        return Commands.run(
            () -> {
                io.setSpeed(speed);
            }, this);
    }
}
