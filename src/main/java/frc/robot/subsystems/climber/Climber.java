package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Climber.CALIBRATION_CURRENT_THRESHOLD;

public class Climber extends SubsystemBase {
  private final String name;
  private final ClimberIO io;

  private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(String name, ClimberIO io) {
      this.name = name;
      this.io = io;
  }

  @Override
  public void periodic() {
      io.updateInputs(inputs);
      Logger.processInputs("Climbers/" + name, inputs);
  }

  /**
   * Returns a trigger that is active when the climber is calibrated.
   *
   * @return a trigger that is active when the climber is calibrated.
   */
  public Trigger calibrated() {
      return new Trigger(() -> inputs.currentDraw > CALIBRATION_CURRENT_THRESHOLD).debounce(0.1);
  }

    /**
     * Creates a command that calibrates the climber.
     *
     * @return a command that calibrates the climber.
     */
  public Command calibrateCommand() {
        return setSpeedCommand(() -> -0.1).until(calibrated());
  }

    /**
     * Creates a command that sets the speed of the climber.
     *
     * @param speed the speed of the climber.
     * @return a command that sets the speed of the climber.
     */
  public Command setSpeedCommand(DoubleSupplier speed) {
      return Commands.run(
          () -> {
              io.setSpeed(speed.getAsDouble());
          }, this);
  }
}
