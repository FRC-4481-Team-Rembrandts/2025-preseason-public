package frc.robot.subsystems.shamper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;


public class Shamper extends SubsystemBase {
    private final ShamperIO io;
    
    private ShamperSetpoints currentSetpoint = new ShamperSetpoints(0, 0, 0, 0);

    private ShamperInputsAutoLogged inputs = new ShamperInputsAutoLogged();

    /** Creates a new Shamper */
    public Shamper(ShamperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shamper", inputs);
    }

    public Command shootToAmp(){
        return setMotorTargets(Constants.Shamper.AMP_SPEEDS);
    }

    public Command shootToSpeaker(DoubleSupplier distanceToGoal){
        return setMotorTargets(getSpeakerSetpoint(distanceToGoal.getAsDouble()));
    }

    public Command shootLobje(){
        return setMotorTargets(Constants.Shamper.LOBJE_SPEEDS);
    }

    public Command eject(){
        return setMotorTargets(Constants.Shamper.EJECT_SPEEDS);
    }
    
    public Command disable(){
        return setMotorTargets(new ShamperSetpoints(0, 0, 0, 0));
    }

    private Command setMotorTargets (ShamperSetpoints setpoints) {
        return Commands.run(() -> {
            currentSetpoint = setpoints;
            io.setTopRollerSpeed(setpoints.topRollerSpeed());
            io.setBottomRollerSpeed(setpoints.bottomRollerSpeed());
            io.setKickerSpeed(setpoints.kickerSpeed());
            io.setPivotPosition(setpoints.pivotPosition());
        }, this);
    }

    private static ShamperSetpoints getSpeakerSetpoint(double distanceToGoal) {
        // TODO add a LUT or InterpolatingTreeMap
        return Constants.Shamper.TEMP_SPEAKER_SPEEDS;
    }


    /**
     * Give me notes notes notes notes
     * True while on setpoint and setpoint is a shooting setpoint
     * False while not on setpoint or the setpoint is an idle setpoint
     * 
     * @return notes please
     */
    public Trigger readyForNote(){
        return new Trigger(() -> {
            //Compare setpoints to actual velocity and check if the setpoint is not an idle setpoint;
            return (Math.abs(inputs.topRollerVelocity - currentSetpoint.topRollerSpeed()) < 100 &&
                    Math.abs(inputs.bottomRollerVelocity - currentSetpoint.bottomRollerSpeed()) < 100 &&
                    Math.abs(inputs.kickerVelocity - currentSetpoint.kickerSpeed()) < 100)
                    && currentSetpoint.topRollerSpeed() != 0;
        });
    }
}
