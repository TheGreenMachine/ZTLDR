package com.team1816.lib.hardware.components.sensor;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static com.team1816.lib.BaseConstants.VisionConstants.*;

/**
 * This class holds the properties needed to configure, use, and simulate a camera using <a href=
 * "https://docs.photonvision.org/">PhotonVision</a>. It also holds a {@link PhotonPoseEstimator}
 * to simplify pose estimation using <a href=
 * "https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html"
 * >AprilTags</a>. Currently, we only support cameras for detecting AprilTags or simply as driver
 * cameras.
 *
 * @see com.team1816.lib.subsystems.Vision
 */
public class Camera {
    public final String name;
    public final Transform3d robotToCamera;
    public final DetectionType detectionType;
    public final PhotonCamera photonCamera;
    public final PhotonPoseEstimator poseEstimator;
    private final SimCameraProperties simCameraProperties;

    /**
     * Constructs a {@link Camera} with the specified configuration values.
     *
     * @param name                The name of the camera to allow referencing a specific camera
     *                            from code.
     * @param photonVisionUIName  The name of the camera used by PhotonVision to identify the
     *                            camera. This <em>MUST</em> match the name of the camera as shown
     *                            in the PhotonVision UI.
     * @param robotToCamera       The {@link Transform3d} from the robot pose to the camera's pose
     * @param detectionType       The type of detection to be performed by the camera.
     * @param simCameraProperties The properties to use when simulating the camera. For the most
     *                            accurate simulation, these should match the behavior of the real
     *                            camera.
     */
    public Camera(
        String name,
        String photonVisionUIName,
        Transform3d robotToCamera,
        DetectionType detectionType,
        SimCameraProperties simCameraProperties
    ) {
        this.name = name;
        this.robotToCamera = robotToCamera;
        this.detectionType = detectionType;
        this.simCameraProperties = simCameraProperties;
        photonCamera = new PhotonCamera(photonVisionUIName);

        // Construct the pose estimator using the robotToCamera for this camera.
        poseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            robotToCamera
        );

        // If we want to detect anything, we can't be in driver mode
        if (detectionType != DetectionType.NONE) {
            photonCamera.setDriverMode(false);
        }
    }

    /**
     * Adds this camera to the provided {@link VisionSystemSim} using the configurations stored by
     * this {@link Camera}.
     *
     * @param sim The {@link VisionSystemSim} to add the camera to.
     */
    public void addToSim(VisionSystemSim sim) {
        PhotonCameraSim cameraSim = new PhotonCameraSim(photonCamera, simCameraProperties, aprilTagFieldLayout);
        cameraSim.enableDrawWireframe(true);
        sim.addCamera(cameraSim, robotToCamera);
    }

    /**
     * Uses this {@link Camera}'s {@link PhotonPoseEstimator} to get {@link EstimatedRobotPose}s
     * for all unread {@link PhotonPipelineResult}s stored on the {@link PhotonCamera} for this
     * {@link Camera}.
     * <p>
     * The pose estimator uses AprilTags seen in the pipeline result and the known position of the
     * camera on the robot to calculate an estimate of the robot's position on the field.
     *
     * @return A list of {@link EstimatedRobotPose}s calculated from all unread pipeline results on
     * this {@link Camera}'s {@link PhotonCamera}.
     */
    public List<EstimatedRobotPose> getEstimatedRobotPosesFromAllUnreadResults() {
        List<EstimatedRobotPose> estimatedRobotPoses = new ArrayList<>();

        // Get position estimates from all the unread PhotonPipelineResults on the PhotonCamera. A
        // PhotonPipelineResult can essentially be thought of as a frame from the camera.
        for (PhotonPipelineResult pipelineResult : photonCamera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> poseEstimate = poseEstimator.estimateCoprocMultiTagPose(pipelineResult);

            // If we weren't able to get a multi-tag estimate (this would happen if there weren't
            // multiple tags visible), try to get a result using lowest ambiguity. (Technically, it
            // doesn't matter that it is using the lowest ambiguity tag because at this point we
            // know there is only one tag, but it has to be some estimation strategy that works
            // with only one tag.)
            if (poseEstimate.isEmpty()) {
                poseEstimate = poseEstimator.estimateLowestAmbiguityPose(pipelineResult);
            }

            // If we got an estimate, add it to the list to return.
            poseEstimate.ifPresent(estimatedRobotPoses::add);
        }
        return estimatedRobotPoses;
    }

    /**
     * Types of detection that a {@link Camera} can perform.
     * <p>
     * We currently only support AprilTag detection or no detection (driver camera). This could
     * eventually be expanded to support colored shape detection and object detection.
     */
    public enum DetectionType {
        /**
         * Don't detect anything. This should be used for a camera only intended for the driver to
         * view, without any vision processing.
         */
        NONE,
        /**
         * Detect <a href=
         * "https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html"
         * >AprilTags</a> to use for robot pose estimation. This corresponds to either the <a href=
         * "https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/detector-types.html"
         * >AprilTag or ArUco pipeline</a> in PhotonVision.
         */
        APRIL_TAG
    }
}
