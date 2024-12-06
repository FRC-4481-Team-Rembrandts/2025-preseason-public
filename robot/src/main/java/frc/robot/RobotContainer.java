/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import static frc.robot.constants.DriveConstants.*;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.auto.AutoSelector;
import frc.lib.input.CommandSwitchProController;
import frc.lib.simulation.VisionEnvironmentSimulator;
import frc.lib.subsystems.drive.*;
import frc.lib.subsystems.vision.RealPhotonVisionIO;
import frc.lib.subsystems.vision.ReplayVisionIO;
import frc.lib.subsystems.vision.SimulatedPhotonVisionIO;
import frc.lib.subsystems.vision.Vision;
import frc.robot.commands.Autos;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystem;
import frc.robot.subsystems.examplesubsystem.ExampleSubsystemIO;
import frc.robot.subsystems.examplesubsystem.RealExampleSubsystemIO;
import frc.robot.subsystems.examplesubsystem.SimulatedExampleSubsystemIO;
import java.lang.reflect.Field;
import java.util.*;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drivetrain;
    private final Vision vision;
    private final ExampleSubsystem exampleSubsystem;
    private final AutoSelector autoSelector;

    // Controller
    private final CommandSwitchProController controller = new CommandSwitchProController(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        drivetrain = createDrive();
        vision = createVision();

        // Switch to define all subsystems
        switch (Constants.MODE) {
            case REAL -> {
                exampleSubsystem = new ExampleSubsystem(new RealExampleSubsystemIO());
            }
            case SIM -> {
                exampleSubsystem = new ExampleSubsystem(new SimulatedExampleSubsystemIO());
            }
            default -> {
                exampleSubsystem = new ExampleSubsystem(new ExampleSubsystemIO() {});
            }
        }
        autoSelector = new AutoSelector(Autos.class, getSubsystemList());

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in
     * {@link CommandGenericHID}'s subclasses for {@link CommandXboxController Xbox}/{@link CommandPS4Controller PS4}
     * controllers or {@link CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.joystickDrive(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                Constants.CONTROLLER_DEADBAND));

        // Add the process vision command as default command for vision
        vision.setDefaultCommand(
                vision.processVision(drivetrain.getRobotPoseSupplier(), drivetrain.getVisionMeasurementConsumer())
                        .ignoringDisable(true));

        exampleSubsystem.setDefaultCommand(exampleSubsystem.disable());
        controller
                .a()
                .onTrue(exampleSubsystem
                        .goToHighSetpoint()
                        .until(exampleSubsystem.onPositionSetpoint().debounce(0.1)));
        controller
                .b()
                .onTrue(exampleSubsystem
                        .goToLowSetpoint()
                        .until(exampleSubsystem.onPositionSetpoint().debounce(0.1)));
        controller.x().whileTrue(exampleSubsystem.enableForward());
        controller.y().whileTrue(exampleSubsystem.enableBackward());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoSelector.getAutonomousCommand();
    }

    private Drive createDrive() {
        return switch (Constants.MODE) {
            case REAL -> new SwerveDrive(
                    TRACK_WIDTH,
                    WHEEL_BASE,
                    MAX_VELOCITY,
                    MAX_ACCELERATION,
                    Constants.LOOPER_DT,
                    ODOMETRY_FREQUENCY,
                    new Pigeon2IMUIO(CAN_ID_IMU),
                    new SparkSwerveModuleIO(
                            new SparkFlex(CAN_ID_FRONT_LEFT_DRIVE, MotorType.kBrushless),
                            new SparkFlex(CAN_ID_FRONT_LEFT_TURN, MotorType.kBrushless),
                            DRIVE_CONFIG_FRONT_LEFT,
                            TURN_CONFIG,
                            DRIVE_FF,
                            TURN_FF),
                    new SparkSwerveModuleIO(
                            new SparkFlex(CAN_ID_FRONT_RIGHT_DRIVE, MotorType.kBrushless),
                            new SparkFlex(CAN_ID_FRONT_RIGHT_TURN, MotorType.kBrushless),
                            DRIVE_CONFIG_FRONT_RIGHT,
                            TURN_CONFIG,
                            DRIVE_FF,
                            TURN_FF),
                    new SparkSwerveModuleIO(
                            new SparkFlex(CAN_ID_BACK_LEFT_DRIVE, MotorType.kBrushless),
                            new SparkFlex(CAN_ID_BACK_LEFT_TURN, MotorType.kBrushless),
                            DRIVE_CONFIG_BACK_LEFT,
                            TURN_CONFIG,
                            DRIVE_FF,
                            TURN_FF),
                    new SparkSwerveModuleIO(
                            new SparkFlex(CAN_ID_BACK_RIGHT_DRIVE, MotorType.kBrushless),
                            new SparkFlex(CAN_ID_BACK_RIGHT_TURN, MotorType.kBrushless),
                            DRIVE_CONFIG_BACK_RIGHT,
                            TURN_CONFIG,
                            DRIVE_FF,
                            TURN_FF));
            case SIM -> new SwerveDrive(
                    TRACK_WIDTH,
                    WHEEL_BASE,
                    MAX_VELOCITY,
                    MAX_ACCELERATION,
                    Constants.LOOPER_DT,
                    ODOMETRY_FREQUENCY,
                    new IMUIO() {},
                    new SimulatedSwerveModuleIO(),
                    new SimulatedSwerveModuleIO(),
                    new SimulatedSwerveModuleIO(),
                    new SimulatedSwerveModuleIO());
            case REPLAY -> new SwerveDrive(
                    TRACK_WIDTH,
                    WHEEL_BASE,
                    MAX_VELOCITY,
                    MAX_ACCELERATION,
                    Constants.LOOPER_DT,
                    ODOMETRY_FREQUENCY,
                    new IMUIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {});
            case XRP -> new DifferentialDrive(
                    0.155,
                    1,
                    1,
                    DriveConstants.ODOMETRY_FREQUENCY,
                    new XRPIMUIO(),
                    new XRPDifferentialDriveIO(XRPDifferentialDriveIO.ModuleSide.LEFT),
                    new XRPDifferentialDriveIO(XRPDifferentialDriveIO.ModuleSide.RIGHT));
        };
    }

    private Vision createVision() {

        switch (Constants.MODE) {
            case REAL:
                return new Vision(
                        Constants.Vision.VISION_FILTER_PARAMETERS,
                        new RealPhotonVisionIO(
                                "camera1", Constants.Vision.robotToCamera1, Constants.Field.APRIL_TAG_FIELD_LAYOUT));
            case SIM:
                VisionEnvironmentSimulator.getInstance()
                        .addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
                VisionEnvironmentSimulator.getInstance().addRobotPoseSupplier(drivetrain.getRobotPoseSupplier());

                return new Vision(
                        Constants.Vision.VISION_FILTER_PARAMETERS,
                        new SimulatedPhotonVisionIO(
                                "camera1",
                                Constants.Vision.robotToCamera1,
                                Constants.Field.APRIL_TAG_FIELD_LAYOUT,
                                Constants.Vision.CAMERA_SIM_PROPERTIES));
            default:
                return new Vision(Constants.Vision.VISION_FILTER_PARAMETERS, new ReplayVisionIO("camera1"));
        }
    }

    /**
     * Detect which fields (variables) are declared in this class that extend SubsystemBase. The detected types are
     * returned as a list
     *
     * @return List of subsystems (SubsystemBase objects)
     */
    private List<SubsystemBase> getSubsystemList() {
        List<SubsystemBase> subsystemList = new ArrayList<>();
        for (Field field : RobotContainer.class.getDeclaredFields()) {
            if (SubsystemBase.class.isAssignableFrom(field.getType())) {
                try {
                    subsystemList.add((SubsystemBase) field.get(this));
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
            }
        }
        return subsystemList;
    }
}
