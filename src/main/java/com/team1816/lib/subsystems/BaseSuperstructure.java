package com.team1816.lib.subsystems;

import com.team1816.lib.BaseRobotState;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static com.team1816.lib.BaseConstants.DrivetrainConstants.defaultStateStdDevs;

public abstract class BaseSuperstructure extends SubsystemBase {
    protected final Swerve swerve;
    protected final Vision vision;

    /**
     * The size (in seconds) of the timestamp bucket used to group nearly-simultaneous
     * vision measurements from multiple cameras for inverse-variance fusion. Measurements
     * whose timestamps fall into the same bucket are fused into a single measurement
     * before being passed to the Kalman filter. This prevents multiple cameras from
     * fighting each other in the filter, which causes pose jitter when stationary.
     */
    private static final double FUSION_TIMESTAMP_BUCKET_SECONDS = 0.1;

    /**
     * Minimum standard deviation (in meters) reported to the Kalman filter for the XY
     * components of a fused vision measurement. Inverse-variance fusion mathematically
     * reduces standard deviation by √N for N independent measurements, but our cameras'
     * disagreements are dominated by systematic bias (mount tolerance, intrinsic
     * calibration) rather than random noise. Without a floor, the fused std dev drops
     * below the bias-limited accuracy, causing the Kalman filter to over-trust the fused
     * measurement and chase bucket-to-bucket variation.
     */
    private static final double FUSED_MIN_XY_STD_DEV_METERS = 0.2;

    /**
     * Minimum standard deviation (in radians) reported to the Kalman filter for the
     * rotation component of a fused vision measurement. See
     * {@link #FUSED_MIN_XY_STD_DEV_METERS}.
     */
    private static final double FUSED_MIN_THETA_STD_DEV_RADIANS = 0.3;

    /**
     * The number of good vision pose estimates we have gotten toward reestablishing our pose
     * estimate since losing an accurate pose estimate.
     */
    private int goodVisionEstimatesSincePoseLoss = 0;

    /**
     * The sum of the variances of the vision standard deviations since losing pose, for performing
     * state standard deviation calculations.
     */
    private Matrix<N3, N1> visionEstimateSincePoseLossVarianceSum = VecBuilder.fill(0, 0, 0);

    protected BaseSuperstructure(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
    }

    @Override
    public void periodic() {
        handlePoseLossFromTilting();
    }

    /**
     * Handles checking if the robot has tilted too far (maybe going over a bump or climbing) to
     * decide if we can no longer trust our robot pose estimate. If we tilt too far, we have either
     * lost wheel contact with the ground or are driving over an uneven surface, and therefore our
     * pose estimate (which includes wheel odometry) will no longer be accurate.
     *
     * Synchronized because {@link #addVisionMeasurementsToDrivetrain()} runs on the vision thread
     * and both methods access {@code goodVisionEstimatesSincePoseLoss} and
     * {@link BaseRobotState#hasAccuratePoseEstimate}.
     */
    private void handlePoseLossFromTilting() {
        // The threshold of how far the robot can be tilted before we decide we don't have an
        // accurate pose estimate. Mainly to account for gyro noise.
        final double tiltPoseLossThresholdRadians = Units.degreesToRadians(5.0);

        if (BaseRobotState.robotTiltRadians > tiltPoseLossThresholdRadians) {
            BaseRobotState.hasAccuratePoseEstimate = false;
            // If we are still tilted after starting to count up, reset our count of good vision
            // estimates toward regaining a good estimate.
            goodVisionEstimatesSincePoseLoss = 0;
        }
    }

    /**
     * Updates the drivetrain's pose estimate by passing in unread vision pose estimates from the
     * {@link Vision} subsystem. Also handles deciding when we have gotten enough good vision
     * estimates to be confident in our combined pose estimate again after losing pose.
     * <p>
     * Before passing measurements to the drivetrain's Kalman filter, nearly-simultaneous
     * measurements from multiple cameras are fused into a single inverse-variance-weighted
     * estimate per time bucket. This prevents cameras from fighting each other in the
     * filter, which causes pose jitter when stationary. See
     * {@link #fuseVisionMeasurements(List)}.
     *
     * Synchronized because this method is called from the vision thread (via
     * {@link com.team1816.lib.BaseRobot}'s addPeriodic) while {@link #handlePoseLossFromTilting()}
     * runs on the main robot thread. Both methods read/write shared state
     * ({@code goodVisionEstimatesSincePoseLoss}, {@code visionEstimateSincePoseLossVarianceSum},
     * and {@link BaseRobotState#hasAccuratePoseEstimate}).
     */
    public void addVisionMeasurementsToDrivetrain() {
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> rawMeasurements = vision
            .getVisionEstimatedPosesWithStdDevs();

        // Fuse nearly-simultaneous measurements from multiple cameras before feeding them
        // to the Kalman filter.
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> visionMeasurements =
            fuseVisionMeasurements(rawMeasurements);

        for (Pair<EstimatedRobotPose, Matrix<N3, N1>> visionMeasurement : visionMeasurements) {
            Matrix<N3, N1> visionStdDevs = visionMeasurement.getSecond();

            if (!BaseRobotState.hasAccuratePoseEstimate && goodVisionEstimatesSincePoseLoss == 0) {
                // If we just lost pose and haven't had any good vision estimates since, assume
                // very little trust in the current state estimate. This will essentially make the
                // next vision measurement added set the pose almost completely.
                swerve.setStateStdDevs(
                    VecBuilder.fill(1000, 1000, 1000)
                );
                // Set the sum of the vision estimate variances to zero for performing state
                // standard deviation calculations.
                visionEstimateSincePoseLossVarianceSum = VecBuilder.fill(0, 0, 0);
            }

            // Add the vision measurements to the drivetrain.
            swerve.addVisionMeasurement(
                visionMeasurement.getFirst().estimatedPose.toPose2d(),
                visionMeasurement.getFirst().timestampSeconds,
                visionStdDevs
            );

            // If we don't currently have a good pose estimate and the vision pose estimate that
            // we just added to the state estimate was good enough, increase our trust in the state
            // estimate and increment our counter toward deciding if we have a good enough estimate
            // again.

            // The maximum x and y standard deviations to allow for a vision estimate to count it
            // as trustworthy enough to work toward reestablishing our pose estimate.
            final double maxTrustworthyTranslationStdDev = 2.0;
            // The maximum rotation standard deviations to allow for a vision estimate to count it
            // as trustworthy enough to work toward reestablishing our pose estimate.
            final double maxTrustworthyRotationStdDev = 4.0;

            if (
                !BaseRobotState.hasAccuratePoseEstimate
                    && visionStdDevs.get(0, 0) <= maxTrustworthyTranslationStdDev
                    && visionStdDevs.get(1, 0) <= maxTrustworthyTranslationStdDev
                    && visionStdDevs.get(2, 0) <= maxTrustworthyRotationStdDev
            ) {
                goodVisionEstimatesSincePoseLoss += 1;

                // The minimum number of good vision pose estimates to wait for until we decide we
                // are confident in our pose estimate again.
                final int minGoodPoseEstimatesUntilConfident = 5;

                // If we've gotten enough good vision pose estimates since losing pose, we know
                // where we are again.
                if (goodVisionEstimatesSincePoseLoss >= minGoodPoseEstimatesUntilConfident) {
                    BaseRobotState.hasAccuratePoseEstimate = true;
                    goodVisionEstimatesSincePoseLoss = 0;
                    // Once we know where we are, set the state standard deviations of the estimate
                    // back to their defaults.
                    swerve.setStateStdDevs(defaultStateStdDevs);
                }
                // If we haven't gotten enough good vision pose estimates yet, increase our trust
                // in the state estimate based on the good estimates we've gotten so far.
                else {
                    // If we treat the state pose estimate as the average of each of the vision pose
                    // estimates since losing pose (the sum of the pose estimates over the number of
                    // pose estimates added), then according to statistics, the standard deviation
                    // should be the square root of the sum of the variances (standard deviations
                    // squared) over the number of pose estimates.
                    visionEstimateSincePoseLossVarianceSum = visionEstimateSincePoseLossVarianceSum
                        .plus(visionStdDevs.elementTimes(visionStdDevs));
                    var stateStdDev = visionEstimateSincePoseLossVarianceSum
                        .elementPower(0.5)
                        .div(goodVisionEstimatesSincePoseLoss);
                    swerve.setStateStdDevs(stateStdDev);
                }
            }
        }
    }

    /**
     * Fuses nearly-simultaneous vision measurements from multiple cameras into a single
     * inverse-variance-weighted measurement per timestamp bucket.
     * <p>
     * Measurements are grouped into buckets of width {@link #FUSION_TIMESTAMP_BUCKET_SECONDS}.
     * Buckets with a single measurement are passed through unchanged. Buckets with multiple
     * measurements are fused using inverse-variance weighting:
     * <ul>
     *     <li>X and Y are weighted averages, weighted by 1/σ² of each measurement.</li>
     *     <li>Rotation is computed as a circular weighted mean (using sin/cos averaging)
     *         so wrap-around at ±π is handled correctly.</li>
     *     <li>Combined standard deviations follow the statistical rule for independent
     *         measurements: σ_fused = sqrt(1 / Σ(1/σ²)).</li>
     * </ul>
     * <p>
     * The returned list is sorted chronologically by timestamp bucket.
     *
     * @param measurements Raw measurements from all cameras.
     * @return One measurement per timestamp bucket, fused across all cameras that
     * contributed to that bucket.
     */
    private static List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> fuseVisionMeasurements(
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> measurements
    ) {
        if (measurements.isEmpty()) {
            return measurements;
        }

        // Group by timestamp bucket.
        Map<Long, List<Pair<EstimatedRobotPose, Matrix<N3, N1>>>> buckets = new HashMap<>();
        for (Pair<EstimatedRobotPose, Matrix<N3, N1>> vm : measurements) {
            long bucketKey = (long) Math.floor(
                vm.getFirst().timestampSeconds / FUSION_TIMESTAMP_BUCKET_SECONDS
            );
            buckets.computeIfAbsent(bucketKey, k -> new ArrayList<>()).add(vm);
        }

        // Process buckets in chronological order.
        List<Long> sortedKeys = new ArrayList<>(buckets.keySet());
        Collections.sort(sortedKeys);

        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> fused = new ArrayList<>(sortedKeys.size());
        for (Long key : sortedKeys) {
            List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> bucket = buckets.get(key);
            if (bucket.size() == 1) {
                fused.add(bucket.get(0));
            }
            else {
                fused.add(fuseBucket(bucket));
            }
        }
        return fused;
    }

    /**
     * Performs inverse-variance fusion of two or more vision measurements captured at
     * nearly the same time.
     *
     * @param bucket Measurements to fuse.
     * @return A single fused measurement.
     */
    private static Pair<EstimatedRobotPose, Matrix<N3, N1>> fuseBucket(
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> bucket
    ) {
        double sumWeightX = 0, sumWeightedX = 0;
        double sumWeightY = 0, sumWeightedY = 0;
        double sumWeightTheta = 0, sumSinWeighted = 0, sumCosWeighted = 0;
        double sumTimestamps = 0;

        List<PhotonTrackedTarget> combinedTargets = new ArrayList<>();

        for (Pair<EstimatedRobotPose, Matrix<N3, N1>> vm : bucket) {
            Pose2d pose2d = vm.getFirst().estimatedPose.toPose2d();
            Matrix<N3, N1> stdDevs = vm.getSecond();

            double sigmaX = stdDevs.get(0, 0);
            double sigmaY = stdDevs.get(1, 0);
            double sigmaTheta = stdDevs.get(2, 0);

            double weightX = 1.0 / (sigmaX * sigmaX);
            double weightY = 1.0 / (sigmaY * sigmaY);
            double weightTheta = 1.0 / (sigmaTheta * sigmaTheta);

            sumWeightX += weightX;
            sumWeightedX += weightX * pose2d.getX();

            sumWeightY += weightY;
            sumWeightedY += weightY * pose2d.getY();

            // Circular weighted mean for theta. Measurements with very large sigmaTheta
            // (e.g. single-tag's 999999) have negligible weight and effectively do not
            // contribute to the fused rotation, which is the desired behavior.
            double thetaRadians = pose2d.getRotation().getRadians();
            sumWeightTheta += weightTheta;
            sumSinWeighted += weightTheta * Math.sin(thetaRadians);
            sumCosWeighted += weightTheta * Math.cos(thetaRadians);

            sumTimestamps += vm.getFirst().timestampSeconds;
            combinedTargets.addAll(vm.getFirst().targetsUsed);
        }

        double fusedX = sumWeightedX / sumWeightX;
        double fusedY = sumWeightedY / sumWeightY;
        // atan2(0, 0) returns 0, which is a safe default if all theta weights were zero.
        double fusedTheta = Math.atan2(sumSinWeighted, sumCosWeighted);

        // Apply floors: mathematical √N reduction is optimal for independent noise,
        // but our cameras' disagreements are dominated by bias (mount tolerance, per-camera
        // calibration) that doesn't average out. Flooring prevents the Kalman filter from
        // over-trusting the fused measurement.
        double fusedSigmaX = Math.max(Math.sqrt(1.0 / sumWeightX), FUSED_MIN_XY_STD_DEV_METERS);
        double fusedSigmaY = Math.max(Math.sqrt(1.0 / sumWeightY), FUSED_MIN_XY_STD_DEV_METERS);
        double fusedSigmaTheta = sumWeightTheta > 0
            ? Math.max(Math.sqrt(1.0 / sumWeightTheta), FUSED_MIN_THETA_STD_DEV_RADIANS)
            : Double.POSITIVE_INFINITY;

        double fusedTimestamp = sumTimestamps / bucket.size();

        Pose3d fusedPose = new Pose3d(new Pose2d(fusedX, fusedY, new Rotation2d(fusedTheta)));

        EstimatedRobotPose fusedEstimate = new EstimatedRobotPose(
            fusedPose,
            fusedTimestamp,
            combinedTargets,
            bucket.get(0).getFirst().strategy
        );

        return Pair.of(
            fusedEstimate,
            VecBuilder.fill(fusedSigmaX, fusedSigmaY, fusedSigmaTheta)
        );
    }
}
