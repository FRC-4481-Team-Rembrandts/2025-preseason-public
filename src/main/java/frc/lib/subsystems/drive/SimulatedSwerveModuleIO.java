package frc.lib.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static frc.lib.hardware.constants.GearRatios.RevRobotics.MaxSwerve.REDUCTION_AZIMUTH;
import static frc.lib.hardware.constants.GearRatios.RevRobotics.MaxSwerve.REDUCTION_HIGH_SPEED;

/**
 * A simulated swerve module for use in physics simulation.
 * The model simulates the physical properties of a NEO Vortex for the drive motor
 * and those of a NEO 550 for the turn motor.
 *
 * <p> This class is not meant to accurately represent a real swerve module.
 * Yet it should be a good enough approximation for testing purposes.
 *
 * <p> The setpoints for the simulated module are derived using computed torque control.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Computed_torque_control">Computed Torque Control</a>
 */
public class SimulatedSwerveModuleIO implements SwerveModuleIO {

    private static final double LOOP_DELTA_TIME = 0.02;
    private static final double WHEEL_RADIUS = Units.Meter.convertFrom(1.5, Units.Inch);

    private static final double DRIVE_INERTIA = 0.15;
    private static final double TURN_INERTIA = 0.04;

    private static final double DRIVE_RATIO = REDUCTION_HIGH_SPEED;
    private static final double TURN_RATIO = REDUCTION_AZIMUTH;

    private static final double DRIVE_NAT_FREQUENCY = 3;
    private static final double TURN_NAT_FREQUENCY = 15;

    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;
    private final DCMotor driveMotor = DCMotor.getNeoVortex(1);
    private final DCMotor turnMotor = DCMotor.getNeo550(1);

    double driveDesiredVelocity = 0; // Desired velocity in rad/s of the wheel
    double turnDesiredPosition = 0; // Desired rotation of the wheel in rad


    /**
     * Creates a new {@code SimulatedSwerveModuleIO}.
     */
    public SimulatedSwerveModuleIO() {
        // The inertia that DCMotorSim takes as input is most likely the inertia felt by the motor
        // Therefore the gear ratio should be taken into account
        driveSim = new DCMotorSim(driveMotor, DRIVE_RATIO, DRIVE_INERTIA / DRIVE_RATIO);
        turnSim = new DCMotorSim(turnMotor, TURN_RATIO, TURN_INERTIA / TURN_RATIO);

        // Random offset when the robot starts to make the simulation more interesting
        Rotation2d arbitraryTurnOffset = new Rotation2d(Math.random() * 2 * Math.PI);
        turnSim.setState(arbitraryTurnOffset.getRadians(), 0);
    }

    /**
     * Updates the system with new inputs from the module.
     *
     * @param inputs The new inputs.
     */
    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveSim.update(LOOP_DELTA_TIME);
        turnSim.update(LOOP_DELTA_TIME);

        // Determine the voltage that needs to be sent to the sim objects using Inverse Dynamics,
        // also known as computed torque control.
        // The desired acceleration is determined based on the error in velocity and position
        // This acceleration is then used to compute the voltage that needs to be sent to the motors
        double driveAppliedVoltage = calculateDriveVoltage();
        double turnAppliedVoltage = calculateTurnVoltage();

        inputs.drivePosition = driveSim.getAngularPositionRad() * WHEEL_RADIUS;
        inputs.driveVelocity = driveSim.getAngularVelocityRadPerSec() * WHEEL_RADIUS;
        inputs.driveAppliedVoltage = driveAppliedVoltage;
        inputs.driveCurrent = driveSim.getCurrentDrawAmps();
        inputs.driveTemperature = 0;

        inputs.turnAbsoluteRotation = new Rotation2d(turnSim.getAngularPositionRad());
        inputs.turnAngularVelocity = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVoltage = turnAppliedVoltage;
        inputs.turnCurrent = turnSim.getCurrentDrawAmps();
        inputs.turnTemperature = 0;


        if (DriverStation.isEnabled()){
            driveSim.setInputVoltage(driveAppliedVoltage);
            turnSim.setInputVoltage(turnAppliedVoltage);
        }
    }

    /**
     * Sets the target drive velocity for the module.
     *
     * @param velocity target drive velocity in m/s.
     */
    @Override
    public void setDriveVelocity(double velocity) {
        driveDesiredVelocity = velocity / WHEEL_RADIUS; //Convert m/s to rad/s
    }

    /**
     * Sets the target turn position for the module.
     *
     * @param rotation target turn position in radians.
     */
    @Override
    public void setTurnRotation(double rotation) {
        turnDesiredPosition = rotation;
    }

    /**
     * Calculates the voltage to be applied to the drive motor.
     * @return The voltage to be applied to the drive motor.
     */
    private double calculateDriveVoltage() {
        double driveAccel = 2 * DRIVE_NAT_FREQUENCY * (driveDesiredVelocity - driveSim.getAngularVelocityRadPerSec());

        double driveAppliedVoltage = driveMotor.rOhms / (DRIVE_RATIO * driveMotor.KtNMPerAmp) * (
                DRIVE_INERTIA * driveAccel + (DRIVE_RATIO * DRIVE_RATIO * driveMotor.KtNMPerAmp) /
                        (driveMotor.rOhms * driveMotor.KvRadPerSecPerVolt) * driveSim.getAngularVelocityRadPerSec()
        );

        driveAppliedVoltage = MathUtil.clamp(
                driveAppliedVoltage,
                -RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage()
        );

        return driveAppliedVoltage;
    }

    /**
     * Calculates the voltage to be applied to the turn motor.
     * @return The voltage to be applied to the turn motor.
     */
    private double calculateTurnVoltage() {
        double turnError = MathUtil.angleModulus(turnDesiredPosition - turnSim.getAngularPositionRad());
        double turnAccel = Math.pow(TURN_NAT_FREQUENCY, 2) * turnError +
                2 * TURN_NAT_FREQUENCY * -turnSim.getAngularVelocityRadPerSec();

        double turnAppliedVoltage = turnMotor.rOhms / (TURN_RATIO * turnMotor.KtNMPerAmp) * (
                TURN_INERTIA * turnAccel + (TURN_RATIO * TURN_RATIO * turnMotor.KtNMPerAmp) /
                        (turnMotor.rOhms * turnMotor.KvRadPerSecPerVolt) * turnSim.getAngularVelocityRadPerSec()
        );
        
        turnAppliedVoltage = MathUtil.clamp(
                turnAppliedVoltage,
                -RobotController.getBatteryVoltage(),
                RobotController.getBatteryVoltage()
        );

        return turnAppliedVoltage;
    }
}
