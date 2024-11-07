package frc.lib.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystems.vision.VisionMeasurement;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A subsystem for controlling a swerve drivetrain.
 */
public class SwerveDrive extends Drive {

    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final IMUIO imuIO;
    private final IMUInputsAutoLogged inputs = new IMUInputsAutoLogged();

    private final double TRACK_WIDTH_X;
    private final double TRACK_WIDTH_Y;
    private final double MAX_VELOCITY;
    private final double MAX_ACCELERATION;
    private final double LOOPER_DT;

    private Rotation2d robotAngle = new Rotation2d();
    private SwerveModulePosition[] previousPositions;

    public final Supplier<Pose2d> robotPoseSupplier;
    public final Consumer<VisionMeasurement> visionMeasurementConsumer;

    /**
     * Creates a new swerve drive subsystem.
     * @param trackWidthX The horizontal distance between the wheels.
     * @param trackWidthY The vertical distance between the wheels.
     * @param maxVelocity The maximum linear velocity of the drivetrain.
     * @param maxAcceleration The maximum linear acceleration of the drivetrain.
     * @param imuIO The input/output that measures the robot angle.
     * @param moduleIOs The input/outputs for the individual swerve modules.
     * @param looperDT The looper time.
     */
    public SwerveDrive(
            double trackWidthX,
            double trackWidthY,
            double maxVelocity,
            double maxAcceleration,
            double looperDT,
            IMUIO imuIO,
            SwerveModuleIO ... moduleIOs
    ){
        super();

        TRACK_WIDTH_X = trackWidthX;
        TRACK_WIDTH_Y = trackWidthY;
        MAX_VELOCITY = maxVelocity;
        MAX_ACCELERATION = maxAcceleration;
        LOOPER_DT = looperDT;

        this.imuIO = imuIO;

        modules = new SwerveModule[moduleIOs.length];
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[moduleIOs.length];

        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(moduleIOs[i], i);
            modulePositions[i] = new SwerveModulePosition();
        }

        previousPositions = modulePositions;

        kinematics = new SwerveDriveKinematics(getModuleTranslations());
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, robotAngle, modulePositions, new Pose2d());

        //Define the consumers and suppliers
        robotPoseSupplier = poseEstimator::getEstimatedPosition;
        visionMeasurementConsumer = (measurement) -> {
            poseEstimator.addVisionMeasurement(measurement.robotPose(),
                    measurement.timestamp(), measurement.stdDevs());
        };
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        imuIO.updateInputs(inputs);
        Logger.processInputs("Swerve/IMU", inputs);

        for (SwerveModule module:modules){
            module.periodic();
        }

        SwerveModulePosition[] modulePositions = getModulePositions();

        if (inputs.connected) {
            robotAngle = inputs.yawPosition;
        } else {
            // estimation, but better than no data at all
            SwerveModulePosition[] deltaPositions = new SwerveModulePosition[modules.length];

            for (int i = 0; i < modules.length; i++) {
                deltaPositions[i] = new SwerveModulePosition(
                        modulePositions[i].distanceMeters - previousPositions[i].distanceMeters,
                        modulePositions[i].angle
                );
            }

            Twist2d twist = kinematics.toTwist2d(deltaPositions);
            robotAngle = robotAngle.plus(new Rotation2d(twist.dtheta));
        }

        poseEstimator.update(robotAngle, modulePositions);
        previousPositions = modulePositions;

        // Log empty data on disable
        if(DriverStation.isDisabled()){
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
        }
    }

    /**
     * Sets the target speeds for the swerve drivetrain. Speeds should be robot relative.
     * @param targetSpeeds The target speeds for the swerve drivetrain.
     */
    @Override
    protected void setTargetSpeeds(ChassisSpeeds targetSpeeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(targetSpeeds, LOOPER_DT);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, getMaxVelocity());

        SwerveModuleState[] optimizedStates = new SwerveModuleState[setpointStates.length];

        // Send data to individual modules
        for( int i = 0; i < modules.length; i++) {
            optimizedStates[i] = SwerveModuleState.optimize(setpointStates[i], modules[i].getState().angle);
            modules[i].setTargetState(optimizedStates[i]);
        }

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
    }

    /**
     * Gets the field relative speeds of the swerve drivetrain.
     * @return The current speeds of the swerve drivetrain.
     */
    @Override
    protected ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                getRobotRelativeSpeeds(),
                poseEstimator.getEstimatedPosition().getRotation()
        );
    }

    /**
     * Gets the robot relative speeds of the swerve drivetrain.
     * @return The current speeds of the swerve drivetrain.
     */
    @Override
    protected ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    /**
     * Gets the current pose of the swerve drivetrain.
     * @return The current pose of the swerve drivetrain.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    protected Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the current states of the swerve modules, which is the velocity and the angle.
     * @return The current states of the individual models.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Retrieve the maximum velocity of the drivetrain.
     * @return The maximum velocity.
     */
    protected double getMaxVelocity() {
        return MAX_VELOCITY;
    }

    /**
     * Retrieve the maximum angular velocity of the drivetrain.
     * @return The maximum angular velocity.
     */
    protected double getMaxAngularVelocity() {
        // Approximation of the radius of the drivetrain
        // Correct if drivetrain is a square
        double driveRadius = Math.hypot(TRACK_WIDTH_X, TRACK_WIDTH_Y);

        // Convert m/s to rad/s
        return getMaxVelocity() / driveRadius;
    }

    /**
     * Retrieve the maximum acceleration of the drivetrain.
     * @return The maximum acceleration.
     */
    protected double getMaxAcceleration(){
        return MAX_ACCELERATION;
    }

    /**
     * Retrieve the maximum angular acceleration of the drivetrain.
     * @return The maximum angular acceleration.
     */
    protected double getMaxAngularAcceleration() {
        // Approximation of the radius of the drivetrain
        // Correct if drivetrain is a square
        double driveRadius = Math.hypot(TRACK_WIDTH_X, TRACK_WIDTH_Y);

        // Convert m/s to rad/s
        return getMaxAcceleration() / driveRadius;
    }


    /**
     * Gets the individual translations of the swerve modules.
     * @return The individual translations of the swerve modules.
     */
    private Translation2d[] getModuleTranslations(){
        return new Translation2d[] {
                new Translation2d(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
                new Translation2d(TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2),
                new Translation2d(-TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
                new Translation2d(-TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2)
        };
    }

    /**
     * Gets the current position for the swerve modules.
     * @return The current positions for the swerve modules.
     */
    private SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Supplier<Pose2d> getRobotPoseSupplier() {
        return this::getPose;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Consumer<VisionMeasurement> getVisionMeasurementConsumer() {
        return  (measurement) -> {
            poseEstimator.addVisionMeasurement(measurement.robotPose(),
                    measurement.timestamp(), measurement.stdDevs());
        };
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Supplier<ChassisSpeeds> getFieldRelativeSpeedsSupplier() {
        return this::getFieldRelativeSpeeds;
    }


    /**
     * {@inheritDoc}
     */
    @Override
    public Command joystickDrive(DoubleSupplier xSupplier,
                                 DoubleSupplier ySupplier,
                                 DoubleSupplier omegaSupplier,
                                 double deadband) {
        return Commands.run(
                () -> {
                    // Apply deadband
                    double linearMagnitude =
                            MathUtil.applyDeadband(
                                    Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), deadband);
                    Rotation2d linearDirection =
                            new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), deadband);

                    // Square values
                    linearMagnitude = linearMagnitude * linearMagnitude;
                    omega = Math.copySign(omega * omega, omega);

                    // Calculate new linear velocity
                    Translation2d linearVelocity =
                            new Pose2d(new Translation2d(), linearDirection)
                                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                                    .getTranslation();

                    // Convert to field relative speeds & send command
                    boolean isFlipped =
                            DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                    this.setTargetSpeeds(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    linearVelocity.getX() * this.getMaxVelocity(),
                                    linearVelocity.getY() * this.getMaxVelocity(),
                                    omega * this.getMaxAngularVelocity(),
                                    isFlipped
                                            ? this.getPose().getRotation().plus(new Rotation2d(Math.PI))
                                            : this.getPose().getRotation()));
                },
                this);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public Command setRobotPose(Pose2d pose){
        return Commands.runOnce(()-> poseEstimator.resetPosition(robotAngle, getModulePositions(), pose));
    }

}
