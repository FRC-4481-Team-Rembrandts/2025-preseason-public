package frc.lib.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.path.TrajectoryFollower;
import frc.lib.subsystems.vision.VisionMeasurement;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A subsystem that represents a drivetrain.
 * Can be used to create commands that control the drivetrain without needing to know the specifics.
 */
public abstract class Drive extends SubsystemBase {
    public Drive() {
        super();
    }
    /**
     * Sets the target speeds for the drivetrain. Speeds should be robot relative.
     * @param targetSpeeds The target speeds for the drivetrain.
     */
    protected abstract void setTargetSpeeds(ChassisSpeeds targetSpeeds);

    /**
     * Gets the robot relative speeds of the drivetrain.
     * @return The robot relative speeds of the drivetrain.
     */
    protected abstract ChassisSpeeds getRobotRelativeSpeeds();

    /**
     * Gets the field relative speeds of the drivetrain.
     * @return The field relative speeds of the drivetrain.
     */
    protected abstract ChassisSpeeds getFieldRelativeSpeeds();

    /**
     * Gets the pose of the drivetrain.
     * @return The pose of the drivetrain.
     */
    protected abstract Pose2d getPose();

    /**
     * Gets the maximum velocity of the drivetrain.
     * @return The maximum velocity of the drivetrain.
     */
    protected abstract double getMaxVelocity();

    /**
     * Gets the maximum angular velocity of the drivetrain.
     * @return The maximum angular velocity of the drivetrain.
     */
    protected abstract double getMaxAngularVelocity();

    /**
     * Gets the maximum acceleration of the drivetrain.
     * @return The maximum acceleration of the drivetrain.
     */
    protected abstract double getMaxAcceleration();

    /**
     * Gets the maximum angular acceleration of the drivetrain.
     * @return The maximum angular acceleration of the drivetrain.
     */
    protected abstract double getMaxAngularAcceleration();

    /**
     * Get the supplier for the current robot pose
     * @return The supplier for the current robot pose
     */
    public abstract Supplier<Pose2d> getRobotPoseSupplier();

    /**
     * Get the consumer for the vision measurements, this is used by the vision subsystem to pass measurements to the drive subsystem
     * @return The consumer for the vision measurements
     */
    public abstract Consumer<VisionMeasurement> getVisionMeasurementConsumer();

    /**
     * Get the supplier for the field relative speeds
     * @return The supplier for the field relative speeds
     */
    public abstract Supplier<ChassisSpeeds> getFieldRelativeSpeedsSupplier();

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public abstract Command joystickDrive(DoubleSupplier xSupplier,
                                          DoubleSupplier ySupplier,
                                          DoubleSupplier omegaSupplier,
                                          double deadband);

    /**
     * Sets the robot to a specified pose.
     * @param pose The pose to reset to.
     */
    public abstract Command setRobotPose(Pose2d pose);

    /**
     * sets the targetSpeeds for a drivetrain to follow a trajectory.
     * @param trajectoryFollower a trajectoryFollower.
     * @return a command that follows a given trajectory and interrupts when the target is reached.
     */
  
    public Command followTrajectory(TrajectoryFollower trajectoryFollower) {
        return Commands.run(
                () -> this.setTargetSpeeds(trajectoryFollower.getSpeeds(this.getPose(),
                        this.getFieldRelativeSpeeds())),
                this
        ).until(trajectoryFollower::isFinished);
    }

    /**
     * Sets the target speeds for the drivetrain. Speeds should be robot relative.
     * @param xSpeed The target speed in the x direction.
     * @param ySpeed The target speed in the y direction.
     * @param angularSpeed The target angular speed.
     */
    Command setTargetSpeeds(double xSpeed, double ySpeed, double angularSpeed){
        return Commands.runOnce(()->   setTargetSpeeds(new ChassisSpeeds(xSpeed, ySpeed, angularSpeed)));
    }

    /**
     * Sets the target speeds for the drivetrain. Speeds should be robot relative.
     * To be used for non-holonomic drivetrains.
     * @param xSpeed The target speed in the x direction.
     * @param angularSpeed The target angular speed.
     */
    Command setTargetSpeeds(double xSpeed, double angularSpeed){
        return Commands.runOnce(()->   setTargetSpeeds(new ChassisSpeeds(xSpeed, 0, angularSpeed)));
    }

}
