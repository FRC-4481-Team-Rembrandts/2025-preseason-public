package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        public double frontRollerPosition;
        public double frontRollerVelocity;
        public double frontRollerAppliedVoltage;
        public double frontRollerCurrentDraw;
        public double frontRollerTemperature;

        public double centerRollerPosition;
        public double centerRollerVelocity;
        public double centerRollerAppliedVoltage;
        public double centerRollerCurrentDraw;
        public double centerRollerTemperature;
    }

    /**
     * Updates the inputs of the subsystem.
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(IntakeInputs inputs){};

    /**
     * Sets speed of front roller motor
     *
     * @param speed in percentage (-1 to 1)
     */
    default void setFrontRollerSpeed(double speed) {};

    /**
     * Sets speed of center roller motor
     *
     * @param speed in percentage (-1 to 1)
     */
    default void setCenterRollerSpeed(double speed) {};

    /**
     * Sets speed of both intake rollers
     * @param speed in percentage (-1 to 1)
     */
    default void setSpeed(double speed){};
}


