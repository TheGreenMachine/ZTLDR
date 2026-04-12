package com.team1816.lib.subsystems.vision.processing;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import static com.team1816.lib.BaseConstants.VisionConstants.StdDevConstants.*;
import static com.team1816.lib.BaseConstants.VisionConstants.aprilTagFieldLayout;

/**
 * Computes pose estimation standard deviations for a single camera.
 */
public class StdDevCalculator {

    private final double alpha;

    private double smoothedArea;
    private double smoothedTagCount;

    private int previousNumTags;
    private boolean initialized;

    public StdDevCalculator(double alpha) {
        this.alpha = Math.max(0.0, Math.min(1.0, alpha));
        this.smoothedArea = 0.1;
        this.smoothedTagCount = 1;
        this.previousNumTags = 0;
        this.initialized = false;
    }

    /**
     * Calculates pose estimation standard deviations based on measurement quality factors.
     * Higher values indicate lower confidence. The result is used to weight this measurement
     * against odometry in the pose estimator.
     *
     * @param avgAmbiguity Average pose ambiguity, lower is better
     * @param avgArea Average tag area as fraction of frame, higher is better
     * @param latencyMs Pipeline latency in milliseconds
     * @param numTags Number of visible AprilTags, must be >= 1
     * @return 3x1 vector of [xStdDev, yStdDev, thetaStdDev] in meters and radians
     */
    public Matrix<N3, N1> calculate(
        double avgAmbiguity,
        double avgArea,
        double latencyMs,
        int numTags
    ) {
        // On first call or tag reacquisition, snap to current values instead of
        // bleeding in slowly from stale defaults.
        boolean reAcquired = (previousNumTags == 0 && numTags > 0);
        if (!initialized || reAcquired) {
            smoothedArea = avgArea;
            smoothedTagCount = numTags;
            initialized = true;
        } else {
            smoothedArea = alpha * avgArea + (1.0 - alpha) * smoothedArea;
            smoothedTagCount = alpha * numTags + (1.0 - alpha) * smoothedTagCount;
        }

        previousNumTags = numTags;

        double latencySeconds = latencyMs / 1000.0;

        double ambiguityFactor = (smoothedTagCount > 1) ? 1.0 : calculateAmbiguityFactor(avgAmbiguity);
        double areaFactor = calculateAreaFactor(smoothedArea);
        double tagCountFactor = calculateTagCountFactor(smoothedTagCount);
        double latencyFactor = calculateLatencyFactor(latencySeconds);

        double xyMultiplier =
            Math.pow(ambiguityFactor, ambiguityWeight) *
            Math.pow(areaFactor, areaWeight) *
            Math.pow(latencyFactor, latencyWeight) *
            tagCountFactor;

        // Theta is less sensitive to area (0.5x) but more sensitive to latency (1.5x)
        // since rotation errors compound more with stale measurements.
        double thetaMultiplier =
            Math.pow(ambiguityFactor, ambiguityWeight) *
            Math.pow(areaFactor, areaWeight * 0.5) *
            Math.pow(latencyFactor, latencyWeight * 1.5) *
            tagCountFactor;

        return VecBuilder.fill(
            baseXYStdDev * xyMultiplier,
            baseXYStdDev * xyMultiplier,
            baseThetaStdDev * thetaMultiplier
        );
    }

    private static double calculateAmbiguityFactor(double ambiguity) {
        ambiguity = Math.max(0.0, Math.min(ambiguity, 0.25));
        double normalized = ambiguity / 0.1;
        return Math.min(1.0 + (normalized * normalized), 8.0);
    }

    private static double calculateAreaFactor(double area) {
        area = Math.max(0.001, Math.min(area, 1.0));
        return Math.min(0.1 / Math.sqrt(area), 5.0);
    }

    private static double calculateLatencyFactor(double latencySeconds) {
        return Math.min(1.0 + (latencySeconds * 5.0), 2.0);
    }

    private static double calculateTagCountFactor(double smoothedTagCount) {
        return (1.0 / Math.log(Math.max(smoothedTagCount, 1.0) + 1.0)) * 1.4;
    }

    private final Matrix<N3, N1> noTrustStdDevs = VecBuilder.fill(
        Double.NaN, Double.NaN, Double.NaN
    );

    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(999999, 999999, 999999);

    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public Matrix<N3, N1> calculateEstimateStandardDeviations(EstimatedRobotPose estimatedRobotPose) {
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
            return singleTagStdDevs.times(1 + (closestDistance * closestDistance / 30));
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
}
