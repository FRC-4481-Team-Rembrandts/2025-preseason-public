package frc.lib.subsystems.vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

/**
 * A class used to interface with a PhotonVision camera
 */
public class RealPhotonVisionIO extends PhotonVisionIO{

    /**
     * Create a new PhotonVisionIO object for a real robot
     * @param cameraName Name of the camera in networktables
     * @param cameraToRobotTransform Transform that represents the translation and rotation from camera to robot centre
     * @param aprilTagFieldLayout Layout of the april tags around the field
     */
    public RealPhotonVisionIO(String cameraName, Transform3d cameraToRobotTransform, AprilTagFieldLayout aprilTagFieldLayout){
        super(cameraName, cameraToRobotTransform, aprilTagFieldLayout);
    }

}







