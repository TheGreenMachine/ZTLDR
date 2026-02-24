package com.team1816.lib.hardware.components.sensor;

import com.team1816.lib.BaseRobotState;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

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
    private final boolean simulateWithPhysicalCamera;
    /**
     * A list of the poses of all AprilTags seen in the latest pipeline result for this camera.
     * <p>
     * Poses are field relative, calculated by transforming from the camera relative AprilTag pose
     * from the pipeline result to a robot relative pose using the {@link #robotToCamera} for this
     * {@link Camera}, and then transforming from the robot relative pose to a field relative pose
     * using the estimated robot pose on the field from the drivetrain.
     */
    private List<Pose3d> seenAprilTagPoses = List.of();
    /**
     * A list of the IDs of all AprilTags seen in the latest pipeline result for this camera.
     */
    private List<Integer> seenAprilTagIDs = List.of();
    /**
     * The latest estimated position from this camera based on a single pipeline result. It is
     * expected for this to jump around a fair amount.
     * <p>
     * This is not to be confused with the pose estimate from the drivetrain that combines all
     * vision estimates and odometry data.
     */
    private Pose3d latestPoseEstimate = Pose3d.kZero;
    /**
     * The latest vision standard deviations from this camera, for logging purposes.
     */
    public Matrix<N3, N1> latestVisionStdDevs = VecBuilder.fill(0.0, 0.0, 0.0);

    /**
     * Constructs a {@link Camera} with the specified configuration values.
     *
     * @param name                       The name of the camera to allow referencing a specific camera
     *                                   from code.
     * @param photonVisionUIName         The name of the camera used by PhotonVision to identify the
     *                                   camera. This <em>MUST</em> match the name of the camera as shown
     *                                   in the PhotonVision UI.
     * @param robotToCamera              The {@link Transform3d} from the robot pose to the camera's pose
     * @param detectionType              The type of detection to be performed by the camera.
     * @param simCameraProperties        The properties to use when simulating the camera. For the most
     *                                   accurate simulation, these should match the behavior of the real
     *                                   camera.
     * @param simulateWithPhysicalCamera If a physical camera will be used for this camera while
     *                                   running the sim.
     */
    public Camera(
        String name,
        String photonVisionUIName,
        Transform3d robotToCamera,
        DetectionType detectionType,
        SimCameraProperties simCameraProperties,
        boolean simulateWithPhysicalCamera
    ) {
        this.name = name;
        this.robotToCamera = robotToCamera;
        this.detectionType = detectionType;
        this.simCameraProperties = simCameraProperties;
        this.simulateWithPhysicalCamera = simulateWithPhysicalCamera;
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
        // If we are using a physical camera in simulation, no need to create a simulated camera.
        if (!simulateWithPhysicalCamera) {
            PhotonCameraSim cameraSim = new PhotonCameraSim(
                photonCamera, simCameraProperties, aprilTagFieldLayout
            );
            cameraSim.enableDrawWireframe(true);
            sim.addCamera(cameraSim, robotToCamera);
        }
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

            // If we got an estimate, add it to the list to return and update the latest pose
            // estimate.
            poseEstimate.ifPresent(estimate -> {
                estimatedRobotPoses.add(estimate);
                latestPoseEstimate = estimate.estimatedPose;
            });

            // Set the seen AprilTag poses and IDs based on the tags from the latest pipeline
            // result.
            List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
            seenAprilTagPoses = targets.stream()
                .map(target ->
                    // Find the AprilTag's pose on the field based on its position relative to the
                    // camera, the position of the camera on the robot, and the current estimated
                    // position of the robot on the field.
                    new Pose3d(BaseRobotState.robotPose)
                        .plus(robotToCamera)
                        .plus(target.getBestCameraToTarget())
                )
                .collect(Collectors.toList());
            seenAprilTagIDs = targets.stream()
                .map(PhotonTrackedTarget::getFiducialId)
                .collect(Collectors.toList());
        }
        return estimatedRobotPoses;
    }

    /**
     * Updates the field on the passed in {@link VisionSystemSim} with the most recent single-frame
     * pose estimation from this camera.
     * <p>
     * If this is a physical camera being used with the sim, this also updates the visible targets
     * and camera position for this camera. This is necessary because PhotonVision does not
     * automatically add the targets and camera position to the field in sim if we are not using a
     * {@link PhotonCameraSim}, which does not make sense to use if we have a physical camera.
     *
     * @param sim The {@link VisionSystemSim} to add the data from this camera to.
     */
    public void updateCameraOnSimField(VisionSystemSim sim) {
        Field2d simField = sim.getDebugField();

        simField.getObject(name + "/visionPoseEstimation").setPose(latestPoseEstimate.toPose2d());

        // For using physical cameras with the simulation, PhotonVision will not automatically add
        // the visible target poses or camera positions to the sim field, so we have to add them
        // manually. This is because the physical cameras shouldn't actually have a
        // PhotonCameraSim (since we don't want it to simulate seeing AprilTags when we want it to
        // be seeing physical AprilTags), but that means PhotonVision won't handle updating
        // positions in the simulation for it at all.
        if (simulateWithPhysicalCamera) {
            // For some reason there is not a method to just add new positions to a Field2d under a
            // name without clearing the old ones, so we have to get the current poses
            // automatically added by PhotonVision from the PhotonCameraSims, add our poses from
            // the physical cameras to that list, and then set the poses on the field to the
            // combined list.
            List<Pose2d> visibleTargetPoses = simField.getObject("visibleTargetPoses").getPoses();
            visibleTargetPoses.addAll(
                // Get a list of Pose2ds from the list of AprilTag Pose3ds.
                // This won't behave in exactly the same way as the cameras with a PhotonCameraSim
                // because those poses are shown relative to the "actual" robot position in the
                // sim, and the seenAprilTagPoses are relative to the estimated robot pose from the
                // drivetrain because they are not only used in sim (so there sometimes is no known
                // "actual" position for them to be calculated from), but this difference is really
                // not important.
                seenAprilTagPoses.stream().map(Pose3d::toPose2d).toList()
            );
            simField.getObject("visibleTargetPoses").setPoses(visibleTargetPoses);

            List<Pose2d> cameras = simField.getObject("cameras").getPoses();
            cameras.add(sim.getRobotPose().plus(robotToCamera).toPose2d());
            simField.getObject("cameras").setPoses(cameras);
        }
    }

    /**
     * Sets up periodic logging for this camera under the specified path.
     *
     * @param logPath The path to log values under.
     */
    public void setUpPeriodicLogging(String logPath) {
        GreenLogger.periodicLogList(
            logPath + "Seen AprilTag Poses", () -> seenAprilTagPoses, Pose3d.class, Pose3d.struct
        );
        GreenLogger.periodicLogList(
            logPath + "Seen AprilTag IDs", () -> seenAprilTagIDs, Integer.class
        );
        GreenLogger.periodicLog(
            logPath + "Latest Pose Estimate", () -> latestPoseEstimate, Pose3d.struct
        );
        GreenLogger.periodicLog(
            logPath + "Latest Vision Standard Deviations",
            () -> latestVisionStdDevs,
            Matrix.getStruct(Nat.N3(), Nat.N1())
        );
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
