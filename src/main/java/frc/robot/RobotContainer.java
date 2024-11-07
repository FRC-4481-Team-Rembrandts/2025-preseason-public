// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.lib.auto.AutoSelector;
import frc.lib.hardware.constants.GearRatios;
import frc.lib.input.CommandSwitchProController;
import frc.lib.subsystems.drive.*;
import frc.lib.subsystems.vision.*;
import frc.robot.commands.Autos;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.PhysicalClimberIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shamper.PhysicalShamperIO;
import frc.robot.subsystems.shamper.Shamper;
import frc.robot.subsystems.shamper.ShamperIO;
import frc.robot.subsystems.shamper.SimulatedShamperIO;
import frc.robot.subsystems.storage.Storage;
import frc.robot.subsystems.intake.PhysicalIntakeIO;
import frc.robot.subsystems.storage.PhysicalStorageIO;
import frc.robot.subsystems.storage.StorageIO;

import java.util.function.BooleanSupplier;
import static frc.robot.Constants.HardwareMap.*;
import static frc.robot.Constants.Climber.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveDrive drivetrain;
  private final Vision vision;
  private final Climber climberLeft;
  private final Climber climberRight;
  private final Intake intake;
  private final Storage storage;
  private final Shamper shamper;

  private final AutoSelector autoSelector;

  // Controller
  private final CommandSwitchProController driver = new CommandSwitchProController(0);
  private final CommandPS4Controller operator = new CommandPS4Controller(1);

  private boolean targetSpeaker = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain = createSwerve();
    vision = createVision();

    //Switch to define all subsystems
    switch (Constants.MODE) {
      case REAL -> {
        shamper = new Shamper(new PhysicalShamperIO());
        climberLeft = new Climber("Left", new PhysicalClimberIO(LEFT_CLIMBER_ID, LEFT_CLIMBER_INVERTED));
        climberRight = new Climber("Right", new PhysicalClimberIO(RIGHT_CLIMBER_ID, RIGHT_CLIMBER_INVERTED));
        intake = new Intake( new PhysicalIntakeIO());
        storage = new Storage( new PhysicalStorageIO());
      }
      case SIM -> {
        shamper = new Shamper(new SimulatedShamperIO());
        climberLeft = new Climber("Left", new ClimberIO() {});
        climberRight = new Climber("Right", new ClimberIO() {});
        intake = new Intake( new IntakeIO() {});
        storage = new Storage( new StorageIO() {});
      }
      default -> {
        climberLeft = new Climber("Left", new ClimberIO() {});
        climberRight = new Climber("Right", new ClimberIO() {});
        intake = new Intake( new IntakeIO(){});
        storage = new Storage( new StorageIO() {});
        shamper = new Shamper(new ShamperIO() {});
      }
    }



    autoSelector = new AutoSelector(Autos.class);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link CommandPS4Controller
   * PS4} controllers or {@link CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand(
            drivetrain.joystickDrive(
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> -driver.getRightX(),
                    Constants.CONTROLLER_DEADBAND
            )
    );

    // Climbers
    climberLeft.calibrated().onTrue(
            climberLeft.setSpeedCommand(operator::getLeftY)
    );
    climberRight.calibrated().onTrue(
            climberRight.setSpeedCommand(operator::getLeftY)
    );

    climberLeft.setDefaultCommand(
            climberLeft.calibrateCommand()
    );
    climberRight.setDefaultCommand(
            climberRight.calibrateCommand()
    );

    // Intake zut
    intake.setDefaultCommand(intake.disableIntakeCommand());
    storage.setDefaultCommand(storage.disableStorageCommand());
    driver.l().onTrue(storage.enableStorageFastCommand().alongWith(intake.enableIntakeCommand())
            .unless(storage.noteFullyInStorage()));
    driver.zl().whileTrue(intake.enableReverseIntakeCommand());

    storage.noteEnteredStorage().onTrue(storage.enableStorageSlowCommand()
            .alongWith(intake.enableIntakeSlowCommand())
            .unless(storage.noteFullyInStorage())
    );
    storage.noteFullyInStorage().onTrue(storage.disableStorageCommand()
            .alongWith(intake.disableIntakeCommand())
            .unless(shamper.readyForNote())
    );


    // Vision
    vision.newMeasurements().whileTrue(drivetrain.addVisionMeasurement(vision::pollMeasurement).ignoringDisable(true));
    vision.setDefaultCommand(vision.setActualRobotPose(drivetrain::getPose));


    // Shamper
    operator.R1().onTrue(Commands.runOnce(() -> targetSpeaker = !targetSpeaker)); // to speak or not to speak

    shamper.setDefaultCommand(shamper.disable());

    driver.r().and(isTargetingSpeaker()).whileTrue(shamper.shootToSpeaker(()->10));
    driver.r().and(isTargetingSpeaker().negate()).whileTrue(shamper.shootToAmp());
    shamper.readyForNote().whileTrue(storage.enableFeedingStorageCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelector.getAutonomousCommand();
  }

  private SwerveDrive createSwerve() {
    return switch (Constants.MODE) {
      case REAL -> new SwerveDrive(
              Constants.Swerve.TRACK_WIDTH_X,
              Constants.Swerve.TRACK_WIDTH_Y,
              Constants.Swerve.MAX_VELOCITY,
              Constants.Swerve.MAX_ACCELERATION,
              Constants.LOOPER_DT,
              new Pigeon2IMUIO(Constants.HardwareMap.IMU_ID),
              new SparkFlexSwerveModuleIO(
                      Constants.HardwareMap.FRONT_LEFT_DRIVE_ID,
                      Constants.HardwareMap.FRONT_LEFT_TURN_ID,
                      Constants.Swerve.DRIVE_PIDF,
                      Constants.Swerve.TURN_PIDF,
                      GearRatios.RevRobotics.MaxSwerve.REDUCTION_HIGH_SPEED,
                      Constants.Swerve.WHEEL_DIAMETER,
                      Constants.Swerve.FRONT_LEFT_INVERTED,
                      Constants.Swerve.CURRENT_LIMIT_DRIVE,
                      Constants.Swerve.CURRENT_LIMIT_TURN,
                      Constants.Swerve.DRIVE_IDLE_MODE,
                      Constants.Swerve.TURN_IDLE_MODE
              ),
              new SparkFlexSwerveModuleIO(
                      Constants.HardwareMap.FRONT_RIGHT_DRIVE_ID,
                      Constants.HardwareMap.FRONT_RIGHT_TURN_ID,
                      Constants.Swerve.DRIVE_PIDF,
                      Constants.Swerve.TURN_PIDF,
                      GearRatios.RevRobotics.MaxSwerve.REDUCTION_HIGH_SPEED,
                      Constants.Swerve.WHEEL_DIAMETER,
                      Constants.Swerve.FRONT_RIGHT_INVERTED,
                      Constants.Swerve.CURRENT_LIMIT_DRIVE,
                      Constants.Swerve.CURRENT_LIMIT_TURN,
                      Constants.Swerve.DRIVE_IDLE_MODE,
                      Constants.Swerve.TURN_IDLE_MODE
              ),
              new SparkFlexSwerveModuleIO(
                      Constants.HardwareMap.BACK_LEFT_DRIVE_ID,
                      Constants.HardwareMap.BACK_LEFT_TURN_ID,
                      Constants.Swerve.DRIVE_PIDF,
                      Constants.Swerve.TURN_PIDF,
                      GearRatios.RevRobotics.MaxSwerve.REDUCTION_HIGH_SPEED,
                      Constants.Swerve.WHEEL_DIAMETER,
                      Constants.Swerve.BACK_LEFT_INVERTED,
                      Constants.Swerve.CURRENT_LIMIT_DRIVE,
                      Constants.Swerve.CURRENT_LIMIT_TURN,
                      Constants.Swerve.DRIVE_IDLE_MODE,
                      Constants.Swerve.TURN_IDLE_MODE
              ),
              new SparkFlexSwerveModuleIO(
                      Constants.HardwareMap.BACK_RIGHT_DRIVE_ID,
                      Constants.HardwareMap.BACK_RIGHT_TURN_ID,
                      Constants.Swerve.DRIVE_PIDF,
                      Constants.Swerve.TURN_PIDF,
                      GearRatios.RevRobotics.MaxSwerve.REDUCTION_HIGH_SPEED,
                      Constants.Swerve.WHEEL_DIAMETER,
                      Constants.Swerve.BACK_RIGHT_INVERTED,
                      Constants.Swerve.CURRENT_LIMIT_DRIVE,
                      Constants.Swerve.CURRENT_LIMIT_TURN,
                      Constants.Swerve.DRIVE_IDLE_MODE,
                      Constants.Swerve.TURN_IDLE_MODE
              )
      );
      case SIM -> new SwerveDrive(
              Constants.Swerve.TRACK_WIDTH_X,
              Constants.Swerve.TRACK_WIDTH_Y,
              Constants.Swerve.MAX_VELOCITY,
              Constants.Swerve.MAX_ACCELERATION,
              Constants.LOOPER_DT,
              new IMUIO() {},
              new SimulatedSwerveModuleIO(),
              new SimulatedSwerveModuleIO(),
              new SimulatedSwerveModuleIO(),
              new SimulatedSwerveModuleIO()
      );
      case REPLAY -> new SwerveDrive(
              Constants.Swerve.TRACK_WIDTH_X,
              Constants.Swerve.TRACK_WIDTH_Y,
              Constants.Swerve.MAX_VELOCITY,
              Constants.Swerve.MAX_ACCELERATION,
              Constants.LOOPER_DT,
              new IMUIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {}
      );
    };
  }

  private Vision createVision(){

    switch(Constants.MODE) {
      case REAL:
        return new Vision(
                Constants.Vision.VISION_FILTER_PARAMETERS,
                new RealPhotonVisionIO("camera1",
                        Constants.Vision.camera1ToRobot,
                        Constants.Field.APRIL_TAG_FIELD_LAYOUT
                )
        );
      case SIM:
        VisionEnvironmentSimulator.getInstance().addAprilTags(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField());
        VisionEnvironmentSimulator.getInstance().addRobotPoseSupplier(drivetrain::getPose);

        return new Vision(
                Constants.Vision.VISION_FILTER_PARAMETERS,
                new SimulatedPhotonVisionIO(
                        "camera1", Constants.Vision.camera1ToRobot,
                        Constants.Field.APRIL_TAG_FIELD_LAYOUT, Constants.Vision.CAMERA_SIM_PROPERTIES
                )
        );
      default:
        return new Vision(
                Constants.Vision.VISION_FILTER_PARAMETERS,
                new VisionIO() {
                }
        );
    }

  }

  Trigger isTargetingSpeaker() {
    return new Trigger(() -> targetSpeaker);
  }
}
