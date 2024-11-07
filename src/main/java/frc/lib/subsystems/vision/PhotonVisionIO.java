package frc.lib.subsystems.vision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

/**
 * A class used to interface with a PhotonVision camera
 */
public abstract class PhotonVisionIO implements VisionIO {
    PhotonCamera camera;
    AprilTagFieldLayout aprilTagFieldLayout;
    Transform3d cameraToRobotTransform;
    String cameraName;

    /**
     * Buffer which keeps track of the robot rotation over the past few seconds
     * This allows us to match a vision estimate (which are determined with some delay) with a robot rotation
     */
    TimeInterpolatableBuffer<Rotation2d> rotationBuffer =
            TimeInterpolatableBuffer.createBuffer(1.5);

    /**
     * Create a new PhotonVisionIO object for either a real or simulated robot
     * @param cameraName Name of the camera in networktables
     * @param cameraToRobotTransform Transform that represents the translation and rotation from camera to robot centre
     * @param aprilTagFieldLayout Layout of the april tags around the field
     */
    public PhotonVisionIO(String cameraName, Transform3d cameraToRobotTransform, AprilTagFieldLayout aprilTagFieldLayout) {
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
        this.cameraToRobotTransform = cameraToRobotTransform;
        this.aprilTagFieldLayout = aprilTagFieldLayout;
    }

    /**
     * Update the inputs of a photon vision system
     * Depending on whether one or multiple tags are visible, the robot pose is estimated differently
     * Multiple alternatives are found for the robot pose, which are stored in the robot pose array
     *
     * @param inputs the inputs to update
     */
    @Override
    public void updateInputs(VisionInputs inputs) {
        //Pass the camera name to the inputs
        inputs.cameraName = cameraName;

        //Prevent errors when camera is disconnected or there are no results
        if (camera == null || !camera.isConnected() || camera.getLatestResult() == null || !camera.getLatestResult().hasTargets()) {
            inputs.hasResults = false;
            return;
        }
        inputs.hasResults = true;

        PhotonPipelineResult latestResult = camera.getLatestResult();
        List<PhotonTrackedTarget> visibleTags = latestResult.getTargets();

        inputs.ambiguityRatio = latestResult.getBestTarget().getPoseAmbiguity();

        //Fill in the tag area and visible ID arrays
        double[] areas = new double[visibleTags.size()];
        int[] ID = new int[visibleTags.size()];
        for (int i = 0; i < visibleTags.size(); i++) {
            areas[i] = visibleTags.get(i).getArea();
            ID[i] = visibleTags.get(i).getFiducialId();

        }
        inputs.tagAreas = areas;
        inputs.visibleTagIDs = ID;

        //Retrieve in the timestamp of the latest measurement
        inputs.timeStamp = latestResult.getTimestampSeconds();

        //Determine the possible robot pose results the camera has determined
        if (visibleTags.size() == 1) {
            inputs.fieldSpaceRobotPoses = retrieveSingleTagEstimates(latestResult);
        } else {
            inputs.fieldSpaceRobotPoses = retrieveMultiTagEstimates(latestResult);
        }
    }

    /**
     * Pass the robot rotation that is measured with the IMU to the vision system
     * This should be updated every loop
     *
     * @param robotRotation Actual rotation of the robot
     */
    @Override
    public void setRobotRotation(Rotation2d robotRotation) {
        //Put the rotation in a buffer
        rotationBuffer.addSample(Timer.getFPGATimestamp(), robotRotation);
    }

    /**
     * Determine several options for the robot pose in field space when only 1 tag is visible
     * This algorithm will discard the rotation of the tag and use the robot rotation instead for improved accuracy
     *
     * @param latestResult latest PhotonPipelineResult
     * @return Array with possible robot poses in field space
     */
    private Pose3d[] retrieveSingleTagEstimates(PhotonPipelineResult latestResult) {
        Pose3d[] possibleRobotposes = new Pose3d[2];
        PhotonTrackedTarget target = latestResult.getBestTarget();

        //Only proceed if the target can be found in the april tag field layout
        //and the robot rotation can be retrieved at the time that the result was determined
        Optional<Pose3d> tagPoseOptional = aprilTagFieldLayout.getTagPose(target.getFiducialId());
        Optional<Rotation2d> robotRotationOptional = rotationBuffer.getSample(latestResult.getTimestampSeconds());
        if (tagPoseOptional.isPresent()
                && robotRotationOptional.isPresent()) {
            Pose3d tagPose = tagPoseOptional.get();
            Rotation3d tagRotation = tagPose.getRotation();
            Rotation2d robotRotation = robotRotationOptional.get();

            //Now convert robot rotation to Rotation3d and subtract it from the tag pose to get the relative rotation between robot and target
            Rotation3d robotToTargetRot = tagRotation.minus(new Rotation3d(0, 0, robotRotation.getRadians()));
            //Now we can include the rotation between the camera and the robot
            Rotation3d cameraToTargetRot = robotToTargetRot.plus(cameraToRobotTransform.getRotation());

            //Now we can combine the rotation of the robot with the translation determined by the camera
            Transform3d[] camToTargetOptions = {
                new Transform3d(target.getBestCameraToTarget().getTranslation(), cameraToTargetRot),
                new Transform3d(target.getAlternateCameraToTarget().getTranslation(), cameraToTargetRot)
            };

            for (int i = 0; i < camToTargetOptions.length; i++) {
                Transform3d camToTarget = camToTargetOptions[i];
                possibleRobotposes[i] = PhotonUtils.estimateFieldToRobotAprilTag(camToTarget, tagPose, cameraToRobotTransform);
            }
        }
        return possibleRobotposes;
    }

    /**
     * Determine several options for the robot pose in field space when multiple tags are visible
     * @param latestResult latest PhotonPipelineResult
     * @return Array with possible robot poses in field space
     */
    private Pose3d[] retrieveMultiTagEstimates(PhotonPipelineResult latestResult) {
        //Retrieve the camera pose in field space represented as transform
        Transform3d bestMultiPose = latestResult.getMultiTagResult().estimatedPose.best;
        Transform3d alternateMultiPose = latestResult.getMultiTagResult().estimatedPose.alt;

        // Convert to field space robot pose and return
        return new Pose3d[]{
            new Pose3d().transformBy(bestMultiPose).transformBy(cameraToRobotTransform),
            new Pose3d().transformBy(alternateMultiPose).transformBy(cameraToRobotTransform)
        };
    }
}