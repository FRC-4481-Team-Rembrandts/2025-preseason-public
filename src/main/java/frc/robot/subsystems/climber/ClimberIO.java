package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberInputs {
        public double position;
        public double velocity;
        public double appliedVoltage;
        public double currentDraw;
        public double temperature;
    }

    /**
     * Updates the inputs of the subsystem.
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(ClimberInputs inputs){};

    /**
     * Sets speed of climber motor
     *
     * @param speed in percentage (-1 to 1)
     */
    default void setSpeed(double speed){};
}


