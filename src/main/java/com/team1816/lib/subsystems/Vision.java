package com.team1816.lib.subsystems;

import com.team1816.lib.BaseRobotState;
import com.team1816.lib.hardware.components.sensor.Camera;
import com.team1816.lib.util.GreenLogger;
import com.team1816.lib.util.RectangularBoundingBox;
import com.team1816.season.Robot;
import com.team1816.season.RobotState;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static com.team1816.lib.BaseConstants.VisionConstants.*;
import static com.team1816.lib.Singleton.factory;
import static edu.wpi.first.units.Units.Inches;

/**
 * This subsystem handles reading and interpreting data from cameras and simulating camera
 * interactions. Currently, we support using cameras for reading <a href=
 * "https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html"
 * >AprilTags</a> to correct our estimate of the robot's position on the field.
 * <p>
 * We use <a href="https://docs.photonvision.org/">PhotonVision</a> for vision processing.
 * <p>
 * Simplified vision pipeline: no proximity filter, no pose loss state machine. The Kalman
 * filter in the drivetrain handles blending vision and odometry based on standard deviations.
 * Single-tag poses with heading that disagrees with gyro by more than 60 degrees are silently
 * discarded as ambiguous reflections.
 */
public class Vision extends SubsystemBase implements ITestableSubsystem {
    private static final String NAME = "vision";
    /**
     * All cameras in the Vision subsystem.
     */
    private final List<Camera> cameras;
    /**
     * The cameras with the {@link Camera.DetectionType#APRIL_TAG APRIL_TAG} detectionType
     */
    private final List<Camera> aprilTagCameras;
    private VisionSystemSim visionSim;
    /**
     * The base standard deviations to use for non-multi-tag estimates.
     * XY uses moderate trust; theta is set to effectively infinity so
     * gyro/odometry controls heading for single-tag (avoids 180-degree flips).
     */
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.5, 0.5, 999999);
    /**
     * The base standard deviations to use for multi-tag estimates
     */
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.1, 0.1, 999999);
    /**
     * The standard deviations to use for an estimate that we should just throw out
     */
    private final Matrix<N3, N1> noTrustStdDevs = VecBuilder.fill(
        Double.NaN, Double.NaN, Double.NaN
    );

    public static final Distance fieldLength = Inches.of(651.2);
    public static final Distance fieldWidth =  Inches.of(317.7);

    private final RectangularBoundingBox acceptableFieldBox = new RectangularBoundingBox(
        new Translation2d(

        ),
        new Translation2d(
            fieldLength,
            fieldWidth
        )
    );
    /**
     * The maximum heading disagreement (in radians) between a single-tag vision estimate
     * and the current robot pose before we discard it as an ambiguous reflection.
     */
    private static final double SINGLE_TAG_MAX_HEADING_ERROR_RADIANS = Units.degreesToRadians(60.0);

    private List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> posesWithStdDevs = new ArrayList<>();
    /**
     * Constructs a Vision subsystem.
     */
    public Vision() {
        cameras = factory.getCameras(NAME);
        aprilTagCameras = cameras.stream().filter(
            camera -> camera.detectionType == Camera.DetectionType.APRIL_TAG
        ).toList();

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("VisionSim");
            visionSim.addAprilTags(aprilTagFieldLayout);
            for (Camera camera : cameras) {
                camera.addToSim(visionSim);
            }
        }

        GreenLogger.periodicLog(
            "vision/Latest Vision Standard Deviations Size",
            () -> posesWithStdDevs.size()
        );

    }

    /**
     * Returns true if the given strategy is a multi-tag strategy.
     */
    private static boolean isMultiTag(PhotonPoseEstimator.PoseStrategy strategy) {
        return strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            || strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO;
    }

    /**
     * Gets all {@link EstimatedRobotPose}s that have not yet been returned by calls to this method
     * from all AprilTag cameras. Estimates are in {@link Pair}s with the standard deviations for
     * the estimate calculated by {@link #calculateEstimateStandardDeviations(EstimatedRobotPose)}.
     * <p>
     * Single-tag estimates whose heading disagrees with the current pose by more than 60 degrees
     * are silently discarded as ambiguous reflections. All other filtering is handled by the
     * standard deviation scaling and the Kalman filter in the drivetrain.
     *
     * @return All unread vision pose estimates with corresponding standard deviations.
     */
    public List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> getVisionEstimatedPosesWithStdDevs() {
        posesWithStdDevs = new ArrayList<>();

        posesWithStdDevs.clear();
        for (Camera camera : aprilTagCameras) {
            var results = camera.getEstimatedRobotPosesFromAllUnreadResults();
            if (RobotState.resetCameraQueue) continue;

            for (EstimatedRobotPose estimatedRobotPose : results) {
                Pose2d visionEstimatedPose2d = estimatedRobotPose.estimatedPose.toPose2d();

                var inField = acceptableFieldBox.withinBounds(estimatedRobotPose.estimatedPose.toPose2d().getTranslation());

                if (!inField){
                    continue;
                }

                // For single-tag estimates, reject if heading disagrees with gyro/odometry
                // by more than 60 degrees. The reflected ambiguous pose from single-tag
                // solvePnP typically has ~180 degree heading error.
                if (!isMultiTag(estimatedRobotPose.strategy)) {
                    double headingError = Math.abs(MathUtil.angleModulus(
                        visionEstimatedPose2d.getRotation()
                            .minus(BaseRobotState.robotPose.getRotation())
                            .getRadians()
                    ));
                    if (headingError > SINGLE_TAG_MAX_HEADING_ERROR_RADIANS) {
                        continue; // Silently discard ambiguous reflection
                    }
                }

                // Calculate the standard deviations for the estimate.
                Matrix<N3, N1> standardDeviations = calculateEstimateStandardDeviations(estimatedRobotPose);

                // If the standard deviations are the noTrustStdDevs, throw the estimate out.
                // Otherwise, add it to the list. The Kalman filter will blend it with
                // odometry proportionally to the std devs.
                if (standardDeviations != noTrustStdDevs) {
                    posesWithStdDevs.add(Pair.of(estimatedRobotPose, standardDeviations));
                }

                // Tell the camera what the standard deviations for its latest estimate were,
                // for logging purposes.
                camera.latestVisionStdDevs = standardDeviations;
            }
        }
        RobotState.resetCameraQueue = false;
        return posesWithStdDevs;
    }

    /**
     * Calculates standard deviations for a PhotonVision {@link EstimatedRobotPose} using a
     * heuristic algorithm.
     * <p>
     * These standard deviations can be used with a pose estimation class (that is,
     * {@link com.ctre.phoenix6.swerve.SwerveDrivetrain CTRE's SwerveDrivetrain} or a {@link
     * edu.wpi.first.math.estimator.PoseEstimator WPILib PoseEstimator}) to change the trust in the
     * vision estimate. A higher standard deviation means less trust.
     *
     * @param estimatedRobotPose The {@link EstimatedRobotPose} from a {@link PhotonPoseEstimator}
     *                           to calculate standard deviations for.
     * @return The calculated standard deviations for the vision pose estimate (x position in
     * meters, y position in meters, and heading in radians). Returns a matrix of {@link Double#NaN}
     * if the estimate should be completely thrown out.
     */
    private Matrix<N3, N1> calculateEstimateStandardDeviations(EstimatedRobotPose estimatedRobotPose) {
        // If we didn't use a multi-tag strategy (single tag or fallback):
        if (!isMultiTag(estimatedRobotPose.strategy)) {
            double closestDistance = Double.MAX_VALUE;
            double lowestAmbiguity = 1;

            for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isEmpty()) continue;

                double distance = tagPose.get().getTranslation().getDistance(
                    estimatedRobotPose.estimatedPose.getTranslation()
                );
                if (distance < closestDistance)
                    closestDistance = distance;

                double ambiguity = target.getPoseAmbiguity();
                if (ambiguity < lowestAmbiguity) {
                    lowestAmbiguity = ambiguity;
                }
            }

            // Reject if too far or too ambiguous.
            if (closestDistance > 6 || lowestAmbiguity > 0.2) {
                return noTrustStdDevs;
            }

            return singleTagStdDevs;
        }

        // Multi-tag: use the base std devs directly without distance or tag-count scaling.
        // Multi-tag PnP accuracy is dominated by systematic bias (camera calibration and
        // mount tolerance) rather than random noise, so scaling by distance and tag count
        // gives a false sense of precision that causes the pose estimate to jump when
        // cameras disagree.
        return multiTagStdDevs;
    }

    @Override
    public void simulationPeriodic() {
        // Update the vision sim with the simulated "actual" robot pose to use as the location that
        // the simulated cameras will simulate seeing from. This pose will show up under the name
        // "Robot" on the vision field. Note that this is different from the "Robot" on the other
        // field, which is the final pose estimate combining odometry and vision data (the
        // "combinedPoseEstimate" on the vision field).
        visionSim.update(BaseRobotState.simActualOrRawOdometryPose);

        // Update the combined odometry and vision pose estimate on the vision field. This should
        // match the robot pose shown on the other field.
        visionSim.getDebugField().getObject("combinedPoseEstimate").setPose(BaseRobotState.robotPose);

        // Clear the visibleTargetPoses and camera poses on the vision sim field for the physical
        // sim cameras before we add the new poses.
        visionSim.getDebugField().getObject("physicalCams/visibleTargetPoses").setPoses();
        visionSim.getDebugField().getObject("physicalCams/cameras").setPoses();

        // Update the vision field with individual camera data.
        for (Camera camera : cameras) {
            camera.updateCameraOnSimField(visionSim);
        }
    }
}
