package frc.robot.subsystems.shamper;

public class SimulatedShamperIO implements ShamperIO {

    double topRollerSpeed = 0;
    double bottomRollerSpeed = 0;
    double kickerSpeed = 0;
    double pivotPosition = 0;

    public SimulatedShamperIO() {
    }

    /**
     * Updates the inputs of the subsystem.
     *
     * @param inputs the inputs of the subsystem.
     */
    @Override
    public void updateInputs(ShamperInputs inputs) {
        inputs.topRollerVelocity = topRollerSpeed;
        inputs.bottomRollerVelocity = bottomRollerSpeed;
        inputs.kickerVelocity = kickerSpeed;
        inputs.pivotAbsoluteEncoderPosition = pivotPosition;
    }

    @Override
    public void setTopRollerSpeed(double speed) {
        topRollerSpeed = speed;
    }

    @Override
    public void setBottomRollerSpeed(double speed) {
        bottomRollerSpeed = speed;
    }

    @Override
    public void setKickerSpeed(double speed) {
        kickerSpeed = speed;
    }

    @Override
    public void setPivotPosition(double position) {
        pivotPosition = position;
    }

}
