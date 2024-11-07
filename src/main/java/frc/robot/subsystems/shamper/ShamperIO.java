package frc.robot.subsystems.shamper;

import org.littletonrobotics.junction.AutoLog;

public interface ShamperIO {
    @AutoLog
    class ShamperInputs {
        public double topRollerVelocity;
        public double topRollerAppliedOutput;
        public double topRollerCurrentDraw;
        public double topRollerTemperature;

        public double bottomRollerVelocity;
        public double bottomRollerAppliedOutput;
        public double bottomRollerCurrentDraw;
        public double bottomRollerTemperature;

        public double kickerVelocity;
        public double kickerAppliedOutput;
        public double kickerCurrentDraw;
        public double kickerTemperature;

        public double pivotRelativeEncoderPosition;
        public double pivotAbsoluteEncoderPosition;
        public double pivotVelocity;
        public double pivotAppliedOutput;
        public double pivotCurrentDraw;
        public double pivotTemperature;
    }

    /**
     * Updates the inputs of the subsystem.
     * @param inputs the inputs of the subsystem.
     */
    default void updateInputs(ShamperInputs inputs){};

    /**
     * Sets speed of top roller motor
     *
     * @param speed in roller RPM
     */
    default void setTopRollerSpeed(double speed) {};

    /**
     * Sets speed of bottom roller motor
     *
     * @param speed in roller RPM
     */
    default void setBottomRollerSpeed(double speed) {};

    /**
     * Sets speed of kicker motor
     *
     * @param speed in roller RPM
     */
    default void setKickerSpeed(double speed) {};

    /**
     * Sets position of pivot motor
     *
     * @param position in degrees of the pivot
     */
    default void setPivotPosition(double position) {};

}
