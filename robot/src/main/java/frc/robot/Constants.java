/*
 * Copyright (c) 2024 FRC 4481 - Team Rembrandts.
 * https://github.com/FRC-4481-Team-Rembrandts.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the WPILib BSD license file in the root directory of this project.
 */
package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.subsystems.vision.VisionFilterParameters;
import frc.lib.util.RunMode;
import org.photonvision.simulation.SimCameraProperties;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {

    public static final RunMode MODE = RunMode.REAL;
    public static final double LOOPER_DT = 0.2;
    public static final double CONTROLLER_DEADBAND = 0.04481;

    public static class HardwareMap {
        public static final int PDH_ID = 1;
    }

    public static class Vision {
        /** Transform3d object that represents the translation and rotation from the robot centre to the camera */
        public static final Transform3d robotToCamera1 = new Transform3d(
                new Translation3d(0.263, 0.140, 0.207),
                new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(-25)));

        public static final SimCameraProperties CAMERA_SIM_PROPERTIES = new SimCameraProperties();

        // Setup the camera sim properties
        static {
            CAMERA_SIM_PROPERTIES.setCalibration(800, 600, Rotation2d.fromDegrees(100));
            CAMERA_SIM_PROPERTIES.setCalibError(0.25, 0.08);
            CAMERA_SIM_PROPERTIES.setFPS(20);
            CAMERA_SIM_PROPERTIES.setAvgLatencyMs(35);
            CAMERA_SIM_PROPERTIES.setLatencyStdDevMs(5);
        }

        public static final VisionFilterParameters VISION_FILTER_PARAMETERS = new VisionFilterParameters(
                0.5,
                1.5,
                Units.Millimeters.of(165),
                0.5,
                Rotation2d.fromDegrees(60),
                Field.FIELD_WIDTH,
                Field.FIELD_LENGTH,
                Units.Centimeter.of(15));
    }

    public static class Field {
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
                AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        public static final Distance FIELD_WIDTH = Units.Meters.of(8.2042);
        public static final Distance FIELD_LENGTH = Units.Meters.of(16.5412);
    }
}
