package com.team1816.lib.subsystems;

import com.team1816.lib.BaseRobotState;
import com.team1816.lib.hardware.components.sensor.Camera;
import com.team1816.lib.util.GreenLogger;
import com.team1816.lib.util.RectangularBoundingBox;
import com.team1816.season.Robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.TreeMap;

import static com.team1816.lib.BaseConstants.VisionConstants.aprilTagFieldLayout;
import static com.team1816.lib.Singleton.factory;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

/**
 * A clean-sheet vision subsystem designed for stable AND responsive pose tracking
 * suitable for shoot-on-the-move applications.
 * <p>
 * <h3>Design philosophy</h3>
 * The single output of this class is a fused vision measurement, fed once per cycle
 * to the drivetrain's Kalman filter. Stability comes from giving the Kalman filter
 * clean, well-characterized measurements — not from smoothing the output. The Kalman
 * filter is the smoother; it just needs honest inputs.
 * <p>
 * Responsiveness comes from the underlying odometry: the drivetrain's pose estimator
 * runs at ~250 Hz from wheel encoders + gyro, and vision arrives at 30-50 Hz to
 * correct accumulated drift. The output, {@link BaseRobotState#robotPose}, is updated
 * at the full odometry rate, so it is fresh enough for shooting on the move.
 * <p>
 * <h3>Key properties</h3>
 * <ul>
 *     <li><b>Theta is gyro-only.</b> Rotation std dev is reported as infinity, so the
 *         Kalman filter ignores vision rotation entirely. This eliminates the dancing
 *         heading problem and exploits the gyro's superior rotational stability.</li>
 *     <li><b>One measurement per cycle.</b> All cameras' results are fused into a
 *         single measurement before reaching the Kalman filter. Cameras can never
 *         "fight" each other inside the filter.</li>
 *     <li><b>Median-based outlier rejection.</b> Inter-camera disagreements are
 *         resolved by rejecting cameras whose pose deviates by more than 40 cm from
 *         the median. A single misconfigured camera cannot pull the consensus.</li>
 *     <li><b>Latency-compensated heading filter.</b> The proximity check compares
 *         vision rotation to the robot's rotation at the vision timestamp, not the
 *         current rotation. Fast rotation no longer triggers spurious rejections.</li>
 *     <li><b>Realistic std dev floor.</b> Fused XY std dev never drops below 10 cm,
 *         matching the bias-limited accuracy of multi-tag PnP in practice.</li>
 * </ul>
 * <p>
 * <h3>Wiring</h3>
 * Call {@link #processFrame()} once per vision-thread cycle. If a measurement is
 * returned, pass its fields to your drivetrain's vision-measurement API. That is
 * the entirety of the integration:
 * <pre>{@code
 *   visionRedux.processFrame().ifPresent(m ->
 *       swerve.addVisionMeasurement(m.pose(), m.timestampSeconds(), m.stdDevs())
 *   );
 * }</pre>
 * <p>
 * For shooter aiming, read {@link BaseRobotState#robotPose} as before. Motion
 * compensation for shoot-on-the-move (leading the shot) is the shooter's
 * responsibility, using {@link BaseRobotState#robotSpeeds}.
 *
 * @see com.team1816.lib.subsystems.Vision (the legacy implementation)
 */
public class VisionRedux extends SubsystemBase implements ITestableSubsystem {
    private static final String NAME = "vision";

    // ============================================================
    // Filter thresholds
    // ============================================================

    /**
     * Maximum age (seconds, FPGA time) of a vision measurement before it is rejected
     * as stale. Anything older than this came from too long ago to usefully correct
     * the current pose.
     */
    private static final double MAX_VISION_AGE_SECONDS = 0.15;

    /**
     * Maximum absolute heading disagreement (radians) between a vision-derived
     * rotation and the robot's gyro-derived rotation at the vision timestamp.
     * Estimates beyond this are treated as ambiguous reflections (single-tag PnP can
     * return a pose flipped 180° in heading) and silently dropped.
     */
    private static final double MAX_HEADING_DISAGREEMENT_RAD = Units.degreesToRadians(20.0);

    /**
     * Maximum distance (meters) to the closest tag for a single-tag estimate to be
     * considered. Beyond this, single-tag PnP is too uncertain to use.
     */
    private static final double MAX_SINGLE_TAG_DISTANCE_METERS = 6.0;

    /**
     * Maximum pose ambiguity for a single-tag estimate to be considered. Above this,
     * the PnP solver is poorly constrained and the result is unreliable.
     */
    private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.2;

    /**
     * During multi-camera fusion, candidates whose translation is more than this far
     * from the median translation are treated as outliers and excluded from the
     * fused result. Chosen to be tight enough to reject misconfigured cameras but
     * loose enough to keep cameras with normal calibration noise.
     */
    private static final double OUTLIER_REJECTION_RADIUS_METERS = 0.40;

    // ============================================================
    // Std dev configuration
    // ============================================================

    /**
     * Floor on the fused-measurement XY std dev. Inverse-variance fusion can
     * mathematically produce arbitrarily small std devs given enough cameras, but
     * the dominant error source is systematic per-camera bias (mount tolerance and
     * intrinsic calibration), which doesn't average out with more cameras. The
     * floor reflects this real-world accuracy limit.
     */
    private static final double FUSED_XY_STD_DEV_FLOOR_METERS = 0.10;

    /**
     * Theta std dev reported to the Kalman filter for any vision measurement.
     * Effectively infinity — gyro owns rotation, vision contributes XY only.
     */
    private static final double VISION_THETA_STD_DEV = 999_999.0;

    // ============================================================
    // Field bounds (REBUILT 2026)
    // ============================================================

    public static final Distance fieldLength = Inches.of(651.2);
    public static final Distance fieldWidth = Inches.of(317.7);

    /**
     * Margin (meters) added to the field rectangle for the bounds rejection check.
     * Allows for legitimate pose-noise-induced reports just outside the painted
     * field perimeter without triggering rejection.
     */
    private static final double FIELD_BOUNDS_MARGIN_METERS = 0.30;

    // ============================================================
    // Pose history (latency compensation)
    // ============================================================

    /**
     * How long (seconds) to retain historical robot poses for latency-compensated
     * proximity checks. Vision frames are typically 30-100 ms old when consumed;
     * 0.5 s of history is comfortably more than enough.
     */
    private static final double POSE_HISTORY_RETENTION_SECONDS = 0.5;

    // ============================================================
    // State
    // ============================================================

    /** All cameras configured in the YAML, including driver cams. */
    private final List<Camera> cameras;

    /** Subset of cameras configured for AprilTag detection. */
    private final List<Camera> aprilTagCameras;

    /** Field rectangle for bounds rejection, with margin baked in. */
    private final RectangularBoundingBox acceptableFieldBox;

    /**
     * Time-ordered buffer of recent robot poses, used to look up where the robot
     * was at the timestamp of an incoming vision measurement.
     */
    private final TreeMap<Double, Pose2d> poseHistory = new TreeMap<>();

    /** Vision sim system, allocated only when running in simulation. */
    private VisionSystemSim visionSim;

    // Diagnostics counters (cycle-local, reset each call to processFrame)
    private int lastFrameRawCandidateCount = 0;
    private int lastFrameAcceptedCandidateCount = 0;
    private int lastFrameContributingCameraCount = 0;
    private double lastFrameMedianDeviationMeters = 0.0;

    // ============================================================
    // Construction
    // ============================================================

    public VisionRedux() {
        cameras = factory.getCameras(NAME);
        aprilTagCameras = cameras.stream()
            .filter(c -> c.detectionType == Camera.DetectionType.APRIL_TAG)
            .toList();

        // Build the field-bounds rectangle. Margin allows legitimate edge-of-field
        // poses to pass; egregious outliers (off the field by half a meter) get
        // rejected.
        double lengthMeters = fieldLength.in(Meters);
        double widthMeters = fieldWidth.in(Meters);
        acceptableFieldBox = new RectangularBoundingBox(
            new Translation2d(-FIELD_BOUNDS_MARGIN_METERS, -FIELD_BOUNDS_MARGIN_METERS),
            new Translation2d(
                lengthMeters + FIELD_BOUNDS_MARGIN_METERS,
                widthMeters + FIELD_BOUNDS_MARGIN_METERS
            )
        );

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("VisionSimRedux");
            visionSim.addAprilTags(aprilTagFieldLayout);
            for (Camera cam : cameras) {
                cam.addToSim(visionSim);
            }
        }

        // Diagnostic logging
        GreenLogger.periodicLog(NAME + "/redux/RawCandidateCount", () -> lastFrameRawCandidateCount);
        GreenLogger.periodicLog(NAME + "/redux/AcceptedCandidateCount", () -> lastFrameAcceptedCandidateCount);
        GreenLogger.periodicLog(NAME + "/redux/ContributingCameraCount", () -> lastFrameContributingCameraCount);
        GreenLogger.periodicLog(NAME + "/redux/MedianDeviationMeters", () -> lastFrameMedianDeviationMeters);
    }

    // ============================================================
    // Public API
    // ============================================================

    /**
     * The fused output of this subsystem for one vision cycle. Pass the fields to
     * the drivetrain's vision-measurement API.
     *
     * @param pose              Field-relative robot pose. Theta is informational only;
     *                          the std dev for theta is set so the Kalman filter
     *                          ignores it.
     * @param timestampSeconds  FPGA timestamp the measurement was captured at. Used
     *                          by the Kalman filter to interpolate odometry to the
     *                          measurement instant.
     * @param stdDevs           [σ_x, σ_y, σ_θ] in meters and radians. σ_θ is
     *                          intentionally enormous so vision does not affect
     *                          rotation in the fused estimate.
     */
    public record VisionMeasurement(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ) {}

    /**
     * Processes all unread vision results from all cameras and returns a single fused
     * measurement, or empty if no usable data was available this cycle.
     * <p>
     * Call this at the cadence of your vision thread (typically 50-100 Hz). Each call
     * collects only the latest unread frame from each camera, so missing a cycle
     * occasionally is harmless.
     *
     * @return The fused vision measurement, or empty if no cameras produced usable
     * data this cycle. Always exactly one measurement when present, never multiple.
     */
    public Optional<VisionMeasurement> processFrame() {
        recordCurrentPoseInHistory();

        // Step 1: Collect a candidate measurement from each camera (the latest
        // unread frame), applying per-camera filters.
        List<Candidate> candidates = collectCandidates();
        lastFrameRawCandidateCount = candidates.size();

        if (candidates.isEmpty()) {
            lastFrameAcceptedCandidateCount = 0;
            lastFrameContributingCameraCount = 0;
            return Optional.empty();
        }

        // Step 2: Reject outliers based on the geometric median of XY positions.
        // This is robust to a single misconfigured camera in a way that weighted
        // means are not.
        List<Candidate> consensus = rejectOutliers(candidates);
        lastFrameAcceptedCandidateCount = consensus.size();
        lastFrameContributingCameraCount = consensus.size();

        if (consensus.isEmpty()) {
            return Optional.empty();
        }

        // Step 3: Fuse the survivors to a single measurement.
        return Optional.of(fuse(consensus));
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim == null) return;
        visionSim.update(BaseRobotState.simActualOrRawOdometryPose);
        visionSim.getDebugField()
            .getObject("combinedPoseEstimate")
            .setPose(BaseRobotState.robotPose);
        for (Camera cam : cameras) {
            cam.updateCameraOnSimField(visionSim);
        }
    }

    // ============================================================
    // Pose history (for latency compensation)
    // ============================================================

    /**
     * Append the current Kalman pose to the history buffer and trim entries older
     * than the retention horizon.
     */
    private void recordCurrentPoseInHistory() {
        double now = Timer.getFPGATimestamp();
        poseHistory.put(now, BaseRobotState.robotPose);
        double cutoff = now - POSE_HISTORY_RETENTION_SECONDS;
        // headMap(cutoff) returns the view of entries strictly before the cutoff.
        if (!poseHistory.isEmpty() && poseHistory.firstKey() < cutoff) {
            poseHistory.headMap(cutoff).clear();
        }
    }

    /**
     * Look up the robot's pose at the given FPGA timestamp by linear interpolation
     * over the recent history buffer. If the timestamp is outside the buffered
     * range, returns the closest available pose. If history is empty, returns the
     * current pose.
     */
    private Pose2d poseAtTimestamp(double timestampSeconds) {
        if (poseHistory.isEmpty()) {
            return BaseRobotState.robotPose;
        }
        var floor = poseHistory.floorEntry(timestampSeconds);
        var ceiling = poseHistory.ceilingEntry(timestampSeconds);
        if (floor == null) return ceiling.getValue();
        if (ceiling == null) return floor.getValue();
        if (floor.getKey().equals(ceiling.getKey())) return floor.getValue();

        double t1 = floor.getKey();
        double t2 = ceiling.getKey();
        double alpha = (timestampSeconds - t1) / (t2 - t1);
        return floor.getValue().interpolate(ceiling.getValue(), alpha);
    }

    // ============================================================
    // Candidate collection (per-camera filtering)
    // ============================================================

    /**
     * Collect the most recent unread vision result from each camera, applying all
     * per-camera filters (latency, field bounds, gyro heading, single-tag quality).
     * Returns one candidate per camera that produced usable data, or zero if none.
     */
    private List<Candidate> collectCandidates() {
        double now = Timer.getFPGATimestamp();
        List<Candidate> candidates = new ArrayList<>(aprilTagCameras.size());

        for (Camera camera : aprilTagCameras) {
            // Drain the camera's queue. We only use the latest in this cycle.
            // PhotonVision queues frames internally; older ones in the queue are
            // dropped because their timestamps will be too old to be useful.
            List<EstimatedRobotPose> results = camera.getEstimatedRobotPosesFromAllUnreadResults();
            EstimatedRobotPose latest = pickLatest(results);
            if (latest == null) continue;

            // --- Filter: stale ---
            double age = now - latest.timestampSeconds;
            if (age > MAX_VISION_AGE_SECONDS) continue;

            // --- Filter: field bounds ---
            Pose2d visionPose2d = latest.estimatedPose.toPose2d();
            if (!acceptableFieldBox.withinBounds(visionPose2d.getTranslation())) continue;

            // --- Filter: latency-compensated heading agreement ---
            // Compare the vision rotation to where the robot WAS (per gyro+odometry)
            // at the vision timestamp, not where it is now. This is the key trick
            // that prevents fast rotation from triggering spurious rejections.
            Pose2d historicalPose = poseAtTimestamp(latest.timestampSeconds);
            double headingError = Math.abs(MathUtil.angleModulus(
                visionPose2d.getRotation().minus(historicalPose.getRotation()).getRadians()
            ));
            if (headingError > MAX_HEADING_DISAGREEMENT_RAD) continue;

            // --- Per-tag analysis ---
            TagAnalysis tags = analyzeTags(latest);
            if (tags.usableTagCount == 0) continue;

            boolean isMultiTag = isMultiTagStrategy(latest.strategy) && latest.targetsUsed.size() > 1;

            // --- Filter: single-tag quality gates ---
            if (!isMultiTag) {
                if (tags.closestDistance > MAX_SINGLE_TAG_DISTANCE_METERS) continue;
                if (tags.minAmbiguity > MAX_SINGLE_TAG_AMBIGUITY) continue;
            }

            candidates.add(new Candidate(
                camera.name,
                visionPose2d,
                latest.timestampSeconds,
                isMultiTag,
                latest.targetsUsed.size(),
                tags.closestDistance,
                tags.avgDistance
            ));

            // Update the per-camera diagnostic field. This is logged elsewhere.
            camera.latestVisionStdDevs = stdDevsForCandidate(candidates.get(candidates.size() - 1));
        }

        return candidates;
    }

    /** Pick the entry with the largest timestamp. Returns null on empty list. */
    private static EstimatedRobotPose pickLatest(List<EstimatedRobotPose> list) {
        EstimatedRobotPose best = null;
        for (EstimatedRobotPose erp : list) {
            if (best == null || erp.timestampSeconds > best.timestampSeconds) best = erp;
        }
        return best;
    }

    private static boolean isMultiTagStrategy(PhotonPoseEstimator.PoseStrategy s) {
        return s == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
            || s == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO;
    }

    /** Result of walking the targets-used list. */
    private record TagAnalysis(
        int usableTagCount,
        double closestDistance,
        double avgDistance,
        double minAmbiguity
    ) {}

    /**
     * Walk the list of tags used to produce an estimate, computing summary stats
     * we'll use for std dev calculation and single-tag quality gates.
     */
    private TagAnalysis analyzeTags(EstimatedRobotPose erp) {
        double closestDistance = Double.MAX_VALUE;
        double sumDistance = 0;
        double minAmbiguity = 1.0;
        int usableTagCount = 0;

        for (PhotonTrackedTarget target : erp.targetsUsed) {
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            double d = tagPose.get().getTranslation().getDistance(erp.estimatedPose.getTranslation());
            closestDistance = Math.min(closestDistance, d);
            sumDistance += d;
            minAmbiguity = Math.min(minAmbiguity, target.getPoseAmbiguity());
            usableTagCount++;
        }

        double avgDistance = usableTagCount > 0 ? sumDistance / usableTagCount : Double.MAX_VALUE;
        if (usableTagCount == 0) closestDistance = Double.MAX_VALUE;
        return new TagAnalysis(usableTagCount, closestDistance, avgDistance, minAmbiguity);
    }

    // ============================================================
    // Outlier rejection
    // ============================================================

    /**
     * Reject candidates whose XY translation is more than
     * {@link #OUTLIER_REJECTION_RADIUS_METERS} from the geometric median XY.
     * <p>
     * The median is robust to a single severely deviating sample (unlike a mean,
     * which gets pulled by outliers). For 5 cameras with one misconfigured, the
     * median sits at one of the four good ones; the bad one's distance from the
     * median is its actual deviation, which exceeds the radius and gets rejected.
     */
    private List<Candidate> rejectOutliers(List<Candidate> candidates) {
        if (candidates.size() <= 1) {
            lastFrameMedianDeviationMeters = 0.0;
            return candidates;
        }

        double[] xs = candidates.stream().mapToDouble(c -> c.pose.getX()).toArray();
        double[] ys = candidates.stream().mapToDouble(c -> c.pose.getY()).toArray();
        Translation2d median = new Translation2d(median(xs), median(ys));

        // Track the worst deviation for diagnostics, before filtering.
        double worstDeviation = 0;
        for (Candidate c : candidates) {
            worstDeviation = Math.max(worstDeviation,
                c.pose.getTranslation().getDistance(median));
        }
        lastFrameMedianDeviationMeters = worstDeviation;

        List<Candidate> kept = new ArrayList<>(candidates.size());
        for (Candidate c : candidates) {
            if (c.pose.getTranslation().getDistance(median) <= OUTLIER_REJECTION_RADIUS_METERS) {
                kept.add(c);
            }
        }
        return kept;
    }

    /** Unweighted median of the input. Modifies a copy of the input. */
    private static double median(double[] arr) {
        double[] copy = Arrays.copyOf(arr, arr.length);
        Arrays.sort(copy);
        int n = copy.length;
        if (n == 0) return 0;
        if ((n & 1) == 1) return copy[n / 2];
        return (copy[n / 2 - 1] + copy[n / 2]) * 0.5;
    }

    // ============================================================
    // Fusion
    // ============================================================

    /**
     * Combine surviving candidates into a single measurement. Position is the
     * inverse-variance weighted mean of XY across candidates; rotation is taken
     * from the historical robot pose at the (averaged) measurement timestamp,
     * because we don't trust vision rotation anyway. The fused std dev is floored
     * at {@link #FUSED_XY_STD_DEV_FLOOR_METERS} to acknowledge bias-limited
     * accuracy of multi-tag PnP in practice.
     */
    private VisionMeasurement fuse(List<Candidate> survivors) {
        // Pre-compute per-candidate std devs and inverse-variance weights.
        double sumWeight = 0;
        double sumWeightedX = 0;
        double sumWeightedY = 0;
        double sumTimestamp = 0;

        for (Candidate c : survivors) {
            Matrix<N3, N1> sd = stdDevsForCandidate(c);
            // Use a single weight for both axes: cameras don't have axis-specific
            // accuracy in practice, and a per-axis weight would over-rotate the
            // fused estimate toward whichever axis happened to be tighter.
            double sigma = sd.get(0, 0);
            double weight = 1.0 / (sigma * sigma);
            sumWeight += weight;
            sumWeightedX += weight * c.pose.getX();
            sumWeightedY += weight * c.pose.getY();
            sumTimestamp += c.timestampSeconds;
        }

        double fusedX = sumWeightedX / sumWeight;
        double fusedY = sumWeightedY / sumWeight;
        double fusedTimestamp = sumTimestamp / survivors.size();

        // Rotation: take from the robot's gyro-integrated pose at the fused
        // timestamp. This value is not actually used by the Kalman filter (theta
        // std dev is infinity), but populating it correctly is useful for logs and
        // any downstream consumer that reads the pose object.
        var historicalRotation = poseAtTimestamp(fusedTimestamp).getRotation();
        Pose2d fusedPose = new Pose2d(fusedX, fusedY, historicalRotation);

        // Fused std dev: classical inverse-variance combining for independent
        // Gaussian samples gives sqrt(1 / sum(1/sigma_i^2)). Floor it to acknowledge
        // bias-limited accuracy.
        double mathematicalStdDev = Math.sqrt(1.0 / sumWeight);
        double flooredXyStdDev = Math.max(mathematicalStdDev, FUSED_XY_STD_DEV_FLOOR_METERS);

        Matrix<N3, N1> fusedStdDevs = VecBuilder.fill(
            flooredXyStdDev,
            flooredXyStdDev,
            VISION_THETA_STD_DEV
        );

        return new VisionMeasurement(fusedPose, fusedTimestamp, fusedStdDevs);
    }

    /**
     * Per-candidate std dev. Captures the realistic accuracy of a single camera's
     * estimate based on tag count and distance. These values are used as inputs to
     * the inverse-variance fusion in {@link #fuse}.
     */
    private Matrix<N3, N1> stdDevsForCandidate(Candidate c) {
        // Base accuracy:
        //   Multi-tag PnP at typical FRC distances: ~3-5 cm under good conditions,
        //     ~10 cm with calibration imperfections.
        //   Single-tag PnP: ~3x worse than multi-tag.
        double baseXy = c.isMultiTag ? 0.05 : 0.20;

        // Distance scaling. Pixel angular resolution × distance produces position
        // uncertainty roughly proportional to distance. Quadratic factor matches
        // empirical fits for FRC cameras at 1-6 m range.
        double distanceFactor = 1.0 + (c.avgDistance * c.avgDistance) / 25.0;

        // Tag-count scaling. Multi-tag with N tags is more accurate than with 2,
        // but the improvement is sub-linear (sqrt) because errors are correlated
        // across tags through camera calibration, not independent.
        double tagCountFactor = c.isMultiTag
            ? 1.0 / Math.sqrt(c.tagCount)
            : 1.0;

        double xy = baseXy * distanceFactor * tagCountFactor;

        return VecBuilder.fill(xy, xy, VISION_THETA_STD_DEV);
    }

    // ============================================================
    // Internal records
    // ============================================================

    /**
     * One camera's contribution to a fusion cycle, after per-camera filtering.
     * Rotation has been validated against the gyro-derived heading at the vision
     * timestamp before we get here.
     */
    private record Candidate(
        String cameraName,
        Pose2d pose,
        double timestampSeconds,
        boolean isMultiTag,
        int tagCount,
        double closestDistance,
        double avgDistance
    ) {}
}
