package com.team1816.lib.subsystems;

import com.team1816.lib.BaseRobotState;
import com.team1816.lib.hardware.components.sensor.Camera;
import com.team1816.lib.util.GreenLogger;
import com.team1816.lib.util.RectangularBoundingBox;
import com.team1816.season.Robot;
import com.team1816.season.RobotState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
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
 */
public class Vision extends SubsystemBase implements ITestableSubsystem {
    private static final double SINGLE_TAG_MAX_HEADING_ERROR_RADIANS = Units.degreesToRadians(60.0);

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
     * The base standard deviations to use for non-multi-tag estimates
     */
    // Ignore single tag data. Previously set to 4, 4, 8.
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.5, 0.5, 999999);
    /**
     * The base standard deviations to use for multi-tag estimates
     */
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.5);
    /**
     * The standard deviations to use for an estimate that we should just throw out
     */
    private final Matrix<N3, N1> noTrustStdDevs = VecBuilder.fill(
        Double.NaN, Double.NaN, Double.NaN
    );
    /**
     * The number of vision pose estimates we have discarded in a row because they were too far off
     * from the combined pose estimate.
     */
    private int consecutiveDiscardedEstimates = 0;

    private boolean isShooting = false;
    private String shootingCameraName = "";

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

        GreenLogger.periodicLog(NAME + "/HasAccuratePoseEstimate", () -> BaseRobotState.hasAccuratePoseEstimate, null);
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
     *
     * @return All unread vision pose estimates with corresponding standard deviations.
     */
    public List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> getVisionEstimatedPosesWithStdDevs() {
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> posesWithStdDevs = new ArrayList<>();
        double highestAverage = Double.MAX_VALUE;
        String currentCameraName = "";
        Pose2d currentPose = Pose2d.kZero;

        // The maximum distance a vision pose estimate can be from the current robot pose
        // estimate to allow the vision estimate to be used.
        double visionEstimateDistanceThresholdMeters = 1.5;
        // The maximum difference the angle of a vision pose estimate can be from the angle
        // of the current robot pose estimate to allow the vision estimate to be used.
        double visionEstimateAngleThresholdRadians = Units.degreesToRadians(15.0);

        for (Camera camera : aprilTagCameras) {
            var results = camera.getEstimatedRobotPosesFromAllUnreadResults();
            if(RobotState.resetCameraQueue) continue;
            for (Camera.TargetsAndPosesResponse response : results) {
                EstimatedRobotPose estimatedRobotPose = response.estimatedRobotPose();
                Pose2d visionEstimatedPose2d = estimatedRobotPose.estimatedPose.toPose2d();

                var inField = acceptableFieldBox.withinBounds(estimatedRobotPose.estimatedPose.toPose2d().getTranslation());

                if (!inField){
                    continue;
                }

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
                
                if (isShooting) {
                    // we are shooting so let's try and lock to a pose from a camera that is multi tag and has the lowest
                    // ambiguity
                    if (!shootingCameraName.isEmpty()) {
                        // already locked on a camera so let's find it and see if it is still multi tag
                        if (camera.name.equals(shootingCameraName)) {
                            if (response.photonTrackedTargets().size() < 2) {
                                // camera is no longer multi tag so find a new one in the next group or remaining cameras
                                shootingCameraName = "";
                                RobotState.shootingPose = RobotState.robotPose;
                            }
                        }
                    } else {
                        // we have not identified a camera so let's hunt for one that is multi tag
                        if (response.photonTrackedTargets().size() > 2) {
                            // found one so average the ambiguity and see if it's lower than what we have
                            var currentAverage = response.photonTrackedTargets().stream().
                                mapToDouble(PhotonTrackedTarget::getPoseAmbiguity).average().orElse(Double.MAX_VALUE);
                            if (currentAverage < highestAverage) {
                                // new lower value so hold on to this
                                highestAverage = currentAverage;
                                currentCameraName = camera.name;
                                currentPose = visionEstimatedPose2d;
                            }
                        }
                    }
                } else {
                    // not shooting so set the shooting pose to the robot pose
                    shootingCameraName = "";
                    RobotState.shootingPose = RobotState.robotPose;
                }

                // Only add the vision measurement to the list to return if it is within the
                // distance and angle thresholds from the current pose estimate. This is to filter
                // out unreasonable estimates caused by pose ambiguity (see here:
                // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html#ambiguity
                // ).
                if (!BaseRobotState.hasAccuratePoseEstimate ||
                    // If we don't currently have an accurate pose estimate, we can't use current
                    // pose estimate to throw out far off vision estimates, so we'll just add the
                    // vision estimate to the list no matter where it is.
                    (
                            // Check if the vision estimate is within the distance threshold of the
                            // current pose estimate.
                            visionEstimatedPose2d.getTranslation().getDistance(
                                BaseRobotState.robotPose.getTranslation()
                            ) < visionEstimateDistanceThresholdMeters
                                // Check if the vision estimate is within the angle threshold of
                                // the current pose estimate. Get the absolute value of the
                                // difference between the angles constrained from -pi radians to pi
                                // radians to find the positive shortest difference.
                                && Math.abs(
                                    MathUtil.angleModulus(
                                        visionEstimatedPose2d.getRotation()
                                            .minus(BaseRobotState.robotPose.getRotation())
                                            .getRadians()
                                    )
                                ) < visionEstimateAngleThresholdRadians
                        )
                ) {
                    // We didn't discard this estimate for being too far off from the combined
                    // estimate, so reset the counter.
                    consecutiveDiscardedEstimates = 0;
                    // Calculate the standard deviations for the estimate.
                    Matrix<N3, N1> standardDeviations = calculateEstimateStandardDeviations(estimatedRobotPose);
                    // If the standard deviations are the noTrustStdDevs, we'll just throw the
                    // estimate out. Otherwise, add the estimate to the list to return.
                    if (standardDeviations != noTrustStdDevs) {
                        posesWithStdDevs.add(Pair.of(estimatedRobotPose, standardDeviations));
                    }
                    // Tell the camera what the standard deviations for its latest estimate were,
                    // for logging purposes.
                    camera.latestVisionStdDevs = standardDeviations;
                }
                // If we discarded enough estimates in a row because they were too far off from the
                // current pose estimate, then it is probably because something is wrong with the
                // current pose estimate (likely due to wheel slippage from hitting something). If
                // this is the case, say that we don't trust the current estimate to allow vision
                // to fully recorrect.
                else {
                    consecutiveDiscardedEstimates ++;
                    // The number of estimates to allow to be discarded before determining that we
                    // have lost a good pose estimate.
                    int discardsBeforePoseLoss = 3;
                    if (consecutiveDiscardedEstimates >= discardsBeforePoseLoss) {
                        BaseRobotState.hasAccuratePoseEstimate = false;
                    }
                }
            }
        }

        if (isShooting) {
            // we are shooting
            if (!currentCameraName.isEmpty() && (currentPose != Pose2d.kZero)) {
                // found a camera so set the shooting pose and camera we used
                shootingCameraName = currentCameraName;
                RobotState.shootingPose = currentPose;
            } else {
                // nothing found so use the robotpose for the shooting pose
                shootingCameraName = "";
                RobotState.shootingPose = RobotState.robotPose;
            }
        } else {
            // not shooting so use the robotpose for the shooting pose
            shootingCameraName = "";
            RobotState.shootingPose = RobotState.robotPose;
        }

        RobotState.resetCameraQueue = false;
        return posesWithStdDevs;
    }

    public void setShootingFlag(boolean isShooting) {
        this.isShooting = isShooting;
    }

    /**
     * Calculates standard deviations for a PhotonVision {@link EstimatedRobotPose} using a
     * heuristic algorithm.
     * <p>
     * These standard deviations can be used with a pose estimation class (that is,
     * {@link com.ctre.phoenix6.swerve.SwerveDrivetrain CTRE's SwerveDrivetrain} or a {@link
     * edu.wpi.first.math.estimator.PoseEstimator WPILib PoseEstimator}) to change the trust in the
     * vision estimate. A higher standard deviation means less trust. The effect of these standard
     * deviations also depends on the state standard deviations of the estimation class, as more
     * trust in the current state of the estimate means that relatively it will trust the vision
     * estimates less.
     * <p>
     * Due to the "close enough," heuristic nature of this algorithm, the standard deviations
     * returned likely do not reflect statistically correct standard deviations of the estimate.
     * Additionally, there are many factors that could affect the accuracy of the vision estimate
     * that are not accounted for by the algorithm. Some known flaws, some of which could
     * potentially be improved in the future, are listed below:
     * <ul>
     *     <li>The base values used for {@link #singleTagStdDevs} and {@link #multiTagStdDevs}
     *     could be adjusted if different values make more sense.</li>
     *     <li>The formula of multiplying the standard deviations by one plus the square of the
     *     distance over 30 could be changed if a different formula is found to be more accurate.
     *     </li>
     *     <li>Estimates from camera frames captured while the robot is moving may be less
     *     trustworthy due to effects like
     *     <a href="https://docs.photonvision.org/en/latest/docs/quick-start/quick-configure.html#apriltags-and-motion-blur-and-rolling-shutter">
     *     motion blur</a>, although it is unclear if this would impact how accurate the estimates
     *     are, or just make it less likely for the camera to pick up on the AprilTags in the first
     *     place.</li>
     *     <li>For non-multi-tag estimates, we could somehow use the pose ambiguity to scale the
     *     standard deviations, not just as a hard cutoff.</li>
     *     <li>For non-multi-tag estimates, four meters may or may not be a good distance cutoff
     *     value. We also could add a distance cutoff for multi-tag, but the value would likely be
     *     different from non-multi-tag, and could depend on closest tag distance, average of
     *     closest two distance, or something else.</li>
     *     <li>For multi-tag estimates, using the average distance of the closest two tags may or
     *     may not make sense. For instance, the distance of the closest tag might matter more than
     *     the distance of the second-closest tag.</li>
     *     <li>For multi-tag estimates, we do not change our trust for tags seen beyond two. Seeing
     *     more tags usually would give a better result, but the exact effect of additional tags is
     *     hard to quantify.</li>
     *     <li>We currently do not distinguish between the x, y, and rotation standard deviations
     *     other than with their base values. These could theoretically be adjusted separately by
     *     using the positions and rotations of the camera and the tags it sees.</li>
     *     <li>For the distance calculations, it would probably be slightly more accurate to use
     *     the position of the camera than that of the robot, but that probably isn't significant.
     *     </li>
     * </ul>
     *
     * @param estimatedRobotPose The {@link EstimatedRobotPose} from a {@link PhotonPoseEstimator}
     *                           to calculate standard deviations for.
     * @return The calculated standard deviations for the vision pose estimate (x position in
     * meters, y position in meters, and heading in radians, although the units are somewhat
     * ambiguous). Returns a matrix of {@link Double#NaN} if the estimate should be completely
     * thrown out.
     */
    private Matrix<N3, N1> calculateEstimateStandardDeviations(EstimatedRobotPose estimatedRobotPose) {
        PhotonPoseEstimator.PoseStrategy strategy = estimatedRobotPose.strategy;

        // If we didn't use a multi-tag strategy. This would happen if we only saw one tag, or
        // decided for some other reason not to use multi-tag.
        if (
            strategy != PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                && strategy != PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO
        ) {
            double closestDistance = Double.MAX_VALUE;
            double lowestAmbiguity = 1;

            // Find the closest distance and lowest ambiguity of the AprilTags seen.
            for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());

                // Theoretically the tag pose could be empty if it saw a tag that isn't in the
                // layout. It couldn't have given us an estimate unless there was at least one
                // valid tag, so we can just ignore any invalid ones.
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

            // If the tag (or best tag if there were multiple) was too far away or the ambiguity
            // (see https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html#ambiguity)
            // was too high, we don't trust this estimate at all.
            if (closestDistance > 6 || lowestAmbiguity > 0.2) {
                return noTrustStdDevs;
            }

            // If we trust the estimate enough to consider it, decrease our trust (increase the
            // standard deviations) with distance, because farther away tags give us less reliable
            // readings. I couldn't tell you why we are using the square of the distance over 30,
            // but that's the equation PhotonVision's example project uses, and it works well
            // enough. Again, this is just a heuristic algorithm.
            int numTags = estimatedRobotPose.targetsUsed.size();

            return singleTagStdDevs.times(1 + (closestDistance * closestDistance / 30)).div(numTags);
        }

        // Otherwise, if we were able to use multi-tag:
        else {
            double closestDistance = Double.MAX_VALUE;
            double secondClosestDistance = Double.MAX_VALUE;

            // Find the two closest distances of the AprilTags seen. Note that ambiguity does not
            // really matter for multi-tag, as having multiple tags can help account for it.
            for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());

                // As before, we could theoretically have a tag not in the layout, but we wouldn't
                // have gotten a multi-tag result unless there were at least two valid tags, so we
                // can just ignore any invalid ones.
                if (tagPose.isEmpty()) continue;

                double distance = tagPose.get().getTranslation().getDistance(
                    estimatedRobotPose.estimatedPose.getTranslation()
                );
                if (distance < closestDistance) {
                    secondClosestDistance = closestDistance;
                    closestDistance = distance;
                }
                else if (distance < secondClosestDistance) {
                    secondClosestDistance = distance;
                }
            }

            // Find the average distance of the closest two tags. If either of the tags needed for
            // a multi-tag estimate was far away, we will have a less reliable reading. However, we
            // do not want to decrease our trust if we see a third (or more), farther away tag, so
            // we only look at the distance of the closest two.
            double averageDistance = (closestDistance + secondClosestDistance) / 2;

            // Decrease trust (increase standard deviations) with average distance of the closest
            // two tags. Use a higher base trust (lower standard deviations) for multi-tag, as it
            // enables a more reliable estimate.
            return multiTagStdDevs.times(1 + (averageDistance * averageDistance / 30));
        }
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
