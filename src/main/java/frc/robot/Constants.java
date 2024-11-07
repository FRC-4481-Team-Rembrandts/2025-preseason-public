// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.lib.control.PIDGains;
import frc.lib.subsystems.vision.VisionFilterParameters;
import frc.lib.util.RunMode;
import frc.lib.control.FeedForwardGains;
import frc.lib.control.PIDFGains;
import frc.robot.subsystems.shamper.ShamperSetpoints;
import org.photonvision.simulation.SimCameraProperties;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final RunMode MODE = RunMode.REAL;
  public static final double LOOPER_DT = 0.2;
  public static final double CONTROLLER_DEADBAND = 0.04481;

  public static class HardwareMap {
    public static final int PDH_ID = 1;

    public static final int IMU_ID = 9;

    public static final int FRONT_LEFT_DRIVE_ID = 11;
    public static final int FRONT_LEFT_TURN_ID = 12;

    public static final int FRONT_RIGHT_DRIVE_ID = 13;
    public static final int FRONT_RIGHT_TURN_ID = 14;

    public static final int BACK_LEFT_DRIVE_ID = 15;
    public static final int BACK_LEFT_TURN_ID = 16;

    public static final int BACK_RIGHT_DRIVE_ID = 17;
    public static final int BACK_RIGHT_TURN_ID = 18;

    public static final int INTAKE_CENTER_MOTOR_ID = 21;
    public static final int INTAKE_FRONT_MOTOR_ID = 23;

    public static final int LEFT_CLIMBER_ID = 41;
    public static final int RIGHT_CLIMBER_ID = 42;

    public static final int STORAGE_ROLLER_MOTOR_ID = 22;
    public static final int FEEDER_ROLLER_MOTOR_ID = 32;

    public static final int LOWER_SENSOR_ID = 0;
    public static final int UPPER_SENSOR_ID = 1;

    public static final int PIVOT_CAN_ID = 31;
    public static final int KICKER_SHOOTER_CAN_ID = 33;
    public static final int TOP_SHOOTER_CAN_ID = 34;
    public static final int BOTTOM_SHOOTER_CAN_ID = 35;

  }

  public static class Swerve {
    public static final boolean FRONT_LEFT_INVERTED = true;
    public static final boolean FRONT_RIGHT_INVERTED = false;
    public static final boolean BACK_LEFT_INVERTED = true;
    public static final boolean BACK_RIGHT_INVERTED = false;

    public static final CANSparkBase.IdleMode DRIVE_IDLE_MODE = CANSparkBase.IdleMode.kBrake;
    public static final CANSparkBase.IdleMode TURN_IDLE_MODE = CANSparkBase.IdleMode.kBrake;

    public static final double WHEEL_DIAMETER = Units.Meter.convertFrom(3, Units.Inch);

    public static final double TRACK_WIDTH_X = 0.45;
    public static final double TRACK_WIDTH_Y = 0.45;

    public static final PIDGains DRIVE_PID = new PIDGains(0, 0, 0);
    public static final FeedForwardGains DRIVE_FF = new FeedForwardGains(0.17491, 2.7538, 0.44218);
    public static final PIDFGains DRIVE_PIDF = new PIDFGains(DRIVE_PID, DRIVE_FF);

    public static final PIDGains TURN_PID = new PIDGains(1, 0, 0.4);
    public static final FeedForwardGains TURN_FF = new FeedForwardGains(0.2, 0.6, 0);
    public static final PIDFGains TURN_PIDF = new PIDFGains(TURN_PID, TURN_FF);

    public static final double MAX_VELOCITY = 4.5;
    public static final double MAX_ACCELERATION = 4;

    public static final int CURRENT_LIMIT_TURN = 23;
    public static final int CURRENT_LIMIT_DRIVE = 60;
  }

  public static class Vision {
    /** Transform3d object that represents the translation and rotation from the camera to robot centre **/
    public static final Transform3d camera1ToRobot = new Transform3d(new Translation3d(0.263, 0.140, 0.207), new Rotation3d(0,Math.toRadians(30),Math.toRadians(-25)));

    public static final SimCameraProperties CAMERA_SIM_PROPERTIES = new SimCameraProperties();
    //Setup the camera sim properties
    static {
      CAMERA_SIM_PROPERTIES.setCalibration(800, 600, Rotation2d.fromDegrees(100));
      CAMERA_SIM_PROPERTIES.setCalibError(0.25, 0.08);
      CAMERA_SIM_PROPERTIES.setFPS(20);
      CAMERA_SIM_PROPERTIES.setAvgLatencyMs(35);
      CAMERA_SIM_PROPERTIES.setLatencyStdDevMs(5);
    }
    public static final VisionFilterParameters VISION_FILTER_PARAMETERS =
            new VisionFilterParameters(0.5, 1.5, Units.Millimeters.of(165),
                    0.5, Rotation2d.fromDegrees(60), Field.FIELD_WIDTH, Field.FIELD_LENGTH, Units.Centimeter.of(15));
  }

  public static class Field {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Measure<Distance> FIELD_WIDTH = Units.Meters.of(8.2042);
    public static final Measure<Distance> FIELD_LENGTH =  Units.Meters.of(16.5412);
  }

  public static class Climber {
    public static final double LOW_SETPOINT = 3;
    public static final double HIGH_SETPOINT = 87;
    public static final double MARGE = 0.05;
    public static final int CURRENT_LIMIT = 30;
    public static final double CALIBRATION_CURRENT_THRESHOLD = 15;
    public static final double CALIBRATION_SPEED = -0.2;
    public static final boolean LEFT_CLIMBER_INVERTED = true;
    public static final boolean RIGHT_CLIMBER_INVERTED = false;
  }

  public static class Intake {
    //Factor to convert between surface speed of intake wheels and surface speed of storage wheels
    public static final double INTAKE_SPEED_FULL = 0.6;
    public static final double INTAKE_SPEED_SLOW = 0.3;
    public static final int CURRENT_LIMIT = 70;
    public static final double INTAKE_SLEW_RATE = 0.5;

  }

  public static class Storage {
    public static final double STORAGE_SPEED_FULL = 0.6;
    public static final double STORAGE_SPEED_SLOW = 0.3;
    public static final double FEEDER_SPEED = 1;
    public static final int CURRENT_LIMIT = 60;
  }

  public static class Shamper {
    public static final ShamperSetpoints AMP_SPEEDS = new ShamperSetpoints(-2000, 2000, 2000, 110);
    public static final ShamperSetpoints LOBJE_SPEEDS = new ShamperSetpoints(1000, 1000, 1000, 50);
    public static final ShamperSetpoints EJECT_SPEEDS = new ShamperSetpoints(1000, 1000, 1000, 31);
    public static final ShamperSetpoints TEMP_SPEAKER_SPEEDS = new ShamperSetpoints(2000,2000, 2000, 60);
    public static final ShamperSetpoints DISABLED = new ShamperSetpoints(0,0, 0, 31);

    public static final int CURRENT_LIMIT_PIVOT = 40;
    public static final int CURRENT_LIMIT_SHOOTER = 60;

    public static final double SHAMPER_SETPOINT_MARGIN = 200;
    public static final double SHAMPER_IDLE_MARGIN = 8000;
    public static final double SHAMPER_SETPOINT_MARGIN_AMP = 1400;
    public static final double SHAMPER_KI_MAX_ACCUM = 0.00001;
    public static final double SHAMPER_KI_DEADZONE = 0.0;

    public static final double CONVERSION_FACTOR_KICKER = 11.0/15.0;
    public static final double CONVERSION_FACTOR_TOP_SHOOTER = 1.667;
    public static final double CONVERSION_FACTOR_BOTTOM_SHOOTER = 1.667;

    public static final double KICKER_KP = 0.00005;
    public static final double KICKER_KI = 0.0000001;
    public static final double KICKER_FF = 1.0/(6600 * CONVERSION_FACTOR_KICKER) * 12;

    public static final double TOP_KP = 0.0003;
    public static final double TOP_KI = 0;// 0.0000001;
    public static final double TOP_FF = 1.0/(6000) * 12;

    public static final double BOTTOM_KP = 0.00004;
    public static final double BOTTOM_KI = 0;//0.000001;
    public static final double BOTTOM_FF = 1.0/(6300) * 12;

    public static final double PIVOT_KP = 0.01;

    public static final int PIVOT_MIN_POSITION = 30;
    public static final int PIVOT_MAX_POSITION = 120;
  }
}
