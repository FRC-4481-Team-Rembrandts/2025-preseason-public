package frc.lib.subsystems.vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

/**
 * A class used to interface with a PhotonVision camera
 */
public class SimulatedPhotonVisionIO extends PhotonVisionIO{

    /**
     * Create a new PhotonVisionIO object for a simulated robot
     * @param cameraName Name of the camera in networktables
     * @param cameraToRobotTransform Transform that represents the translation and rotation from camera to robot centre
     * @param aprilTagFieldLayout Layout of the april tags around the field
     * @param simCameraProperties Properties of the simulated PhotonVision camera
     */
    public SimulatedPhotonVisionIO(String cameraName, Transform3d cameraToRobotTransform, AprilTagFieldLayout aprilTagFieldLayout, SimCameraProperties simCameraProperties){
        super(cameraName, cameraToRobotTransform, aprilTagFieldLayout);
        PhotonCameraSim camSim = new PhotonCameraSim(camera, simCameraProperties);
        VisionEnvironmentSimulator.getInstance().addCamera(camSim, cameraToRobotTransform.inverse());
    }

}







