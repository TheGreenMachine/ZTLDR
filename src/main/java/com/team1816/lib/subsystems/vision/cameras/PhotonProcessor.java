package com.team1816.lib.subsystems.vision.cameras;

import com.ctre.phoenix6.Utils;
import com.team1816.lib.BaseRobotState;
import com.team1816.lib.subsystems.vision.processing.StdDevCalculator;
import com.team1816.lib.subsystems.vision.results.ApriltagResult;
import com.team1816.lib.subsystems.vision.results.ResultInterface;
import com.team1816.lib.subsystems.vision.results.TrackedTag;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;

import static com.team1816.lib.BaseConstants.VisionConstants.maxTagsPerFrame;

/**
 * A processor backed by PhotonVision's camera and pose estimator.
 */
public class PhotonProcessor implements ProcessorInterface {

    private final String cameraName;
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator poseEstimator;
    private final StdDevCalculator stdDevCalculator;
    private final PhotonCameraSim cameraSim;

    private final double processingFrequencyHz;
    private final int maxQueueSize;

    // Fixed-camera field (null when turreted)
    private final Transform3d fixedCameraTransform;

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue = new ConcurrentLinkedQueue<>();
    private final AtomicInteger queueSize = new AtomicInteger(0);

    private final Notifier processingNotifier;
    private volatile boolean isRunning = false;

    private final List<TrackedTag> tagList = new ArrayList<>(maxTagsPerFrame);
    private final TrackedTag[][] tagCache;
    private final ApriltagResult[] resultCache;
    private int resultCacheIndex = 0;

    private final Matrix<N3, N1> stdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
    private final Matrix<N3, N1> scaledStdDevScratch = VecBuilder.fill(0.0, 0.0, 0.0);

    private List<Pose3d> seenAprilTagPoses = List.of();
    /**
     * A list of the IDs of all AprilTags seen in the latest pipeline result for this camera.
     */
    private List<Integer> seenAprilTagIDs = List.of();

    private Matrix<N3, N1> originalCalc = VecBuilder.fill(0.1, 0.1, 0.1);;

    private Matrix<N3, N1> greenCalc = VecBuilder.fill(0.1, 0.1, 0.1);;

    public PhotonProcessor(
        String name,
        AprilTagFieldLayout layout,
        Transform3d transform,
        StdDevCalculator calculator,
        SimCameraProperties simProperties,
        double frequencyHz
    ) {
        this.cameraName = name;
        this.photonCamera = new PhotonCamera(cameraName);
        this.fieldLayout = layout;
        this.fixedCameraTransform = transform;
        this.stdDevCalculator = calculator;
        this.processingFrequencyHz = frequencyHz;
        this.maxQueueSize = computeMaxQueueSize(frequencyHz);

        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, transform);
        this.cameraSim = (simProperties != null)
            ? new PhotonCameraSim(photonCamera, simProperties)
            : null;


        this.resultCache = initResultCache(maxQueueSize);
        this.tagCache = new TrackedTag[maxQueueSize][maxTagsPerFrame];
        initTagCache();

        this.processingNotifier = new Notifier(this::process);
        this.processingNotifier.setName("Photon-" + cameraName);

        setUpPeriodicLogging("VisionSubSystem/cameras/" + name + "/");
    }

    private static ApriltagResult[] initResultCache(int size) {
        ApriltagResult[] cache = new ApriltagResult[size];
        for (int i = 0; i < size; i++) {
            cache[i] = new ApriltagResult();
        }
        return cache;
    }

    private void initTagCache() {
        for (int i = 0; i < maxQueueSize; i++) {
            for (int j = 0; j < maxTagsPerFrame; j++) {
                tagCache[i][j] = new TrackedTag(0, 0.0, 0.0);
            }
        }
    }

    //#region Processing

    @Override
    public void start() {
        isRunning = true;
        processingNotifier.startPeriodic(1.0 / processingFrequencyHz);
    }

    @Override
    public void stop() {
        isRunning = false;
        processingNotifier.stop();
        resultQueue.clear();
        queueSize.set(0);
    }

    @Override
    public void process() {
        if (!isRunning) return;

        List<PhotonPipelineResult> rawResults = photonCamera.getAllUnreadResults();

        for (int i = 0, n = rawResults.size(); i < n; i++) {
            PhotonPipelineResult rawResult = rawResults.get(i);

            ApriltagResult result = processApriltagResult(rawResult);
            if (result != null) {
                while (queueSize.get() >= maxQueueSize) {
                    if (resultQueue.poll() != null) queueSize.decrementAndGet();
                }
                resultQueue.add(result);
                queueSize.incrementAndGet();
            }
        }
    }


    private ApriltagResult processApriltagResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return null;

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        int targetCount = targets.size();
        double avgAmbiguity = calculateAmbiguity(targets);
        double avgArea = calculateAverageArea(targets);

        seenAprilTagPoses = targets.stream()
            .map(target ->
                // Find the AprilTag's pose on the field based on its position relative to the
                // camera, the position of the camera on the robot, and the current estimated
                // position of the robot on the field.
                new Pose3d(BaseRobotState.robotPose)
                    .plus(fixedCameraTransform)
                    .plus(target.getBestCameraToTarget())
            )
            .collect(Collectors.toList());
        seenAprilTagIDs = targets.stream()
            .map(PhotonTrackedTarget::getFiducialId)
            .collect(Collectors.toList());

        Optional<EstimatedRobotPose> poseOpt = (targetCount > 1)
            ? poseEstimator.estimateCoprocMultiTagPose(rawResult)
            : poseEstimator.estimateLowestAmbiguityPose(rawResult);

        if (poseOpt.isEmpty()) return null;

        EstimatedRobotPose estimatedRobotPose = poseOpt.get();


        Pose2d poseEstimate = estimatedRobotPose
            .estimatedPose
            .toPose2d();

        double timestamp = Utils.fpgaToCurrentTime(estimatedRobotPose.timestampSeconds);
        double latency = rawResult.metadata.getLatencyMillis();

        originalCalc = stdDevCalculator.calculate(
            avgAmbiguity, avgArea, latency, targetCount
        );

        for (int i = 0; i < 3; i++) {
            stdDevs.set(i, 0, originalCalc.get(i, 0));
        }

        Matrix<N3, N1> finalStdDevs = stdDevs;

        TrackedTag[] currentFrameTagCache = tagCache[resultCacheIndex];
        tagList.clear();

        for (int i = 0; i < Math.min(targetCount, maxTagsPerFrame); i++) {
            PhotonTrackedTarget target = targets.get(i);
            currentFrameTagCache[i].set(
                target.getFiducialId(),
                target.getArea(),
                target.getPoseAmbiguity()
            );
            tagList.add(currentFrameTagCache[i]);
        }

        ApriltagResult result = resultCache[resultCacheIndex];
        resultCacheIndex = (resultCacheIndex + 1) % maxQueueSize;

        result.set(
            cameraName,
            timestamp,
            latency,
            poseEstimate,
            finalStdDevs,
            tagList,
            avgAmbiguity,
            avgArea
        );

        return result;
    }

    //#endregion

    public void setUpPeriodicLogging(String logPath) {
        GreenLogger.periodicLogList(
            logPath + "Seen AprilTag Poses", () -> seenAprilTagPoses, Pose3d.class, Pose3d.struct
        );
        GreenLogger.periodicLogList(
            logPath + "Seen AprilTag IDs", () -> seenAprilTagIDs, Integer.class
        );
        GreenLogger.periodicLog(
            logPath + "Camera Pose",
            () -> new Pose3d(BaseRobotState.robotPose).plus(fixedCameraTransform),
            Pose3d.struct
        );
        GreenLogger.periodicLog(
            logPath + "Their Calc",
            () -> originalCalc,
            Matrix.getStruct(Nat.N3(), Nat.N1())
        );
        GreenLogger.periodicLog(
            logPath + "Our Calc",
            () -> greenCalc,
            Matrix.getStruct(Nat.N3(), Nat.N1())
        );
    }
    //#region Calculation

    private static double calculateAmbiguity(List<PhotonTrackedTarget> targets) {
        if (targets.size() == 1) return targets.get(0).getPoseAmbiguity();
        double min = 0.01;
        for (int i = 0, n = targets.size(); i < n; i++) {
            double a = targets.get(i).getPoseAmbiguity();
            if (a >= 0 && a < min) min = a;
        }
        return min / Math.sqrt(targets.size());
    }

    private static double calculateAverageArea(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 0.0;
        double sum = 0.0;
        for (int i = 0, n = targets.size(); i < n; i++) sum += targets.get(i).getArea();
        return sum / targets.size();
    }

    private static void fillScaledMultiTagStdDevs(
        Matrix<N3, N1> source,
        Matrix<N3, N1> destination,
        int n
    ) {
        double s = 1.0 / Math.sqrt(n);
        destination.set(0, 0, source.get(0, 0) * s);
        destination.set(1, 0, source.get(1, 0) * s);
        destination.set(2, 0, source.get(2, 0) * s);
    }

    private static int computeMaxQueueSize(double hz) {
        return Math.max(4, (int) Math.ceil(hz / 10.0));
    }

    //#endregion

    //#region ProcessorInterface

    @Override public String getCameraName() {
        return cameraName;
    }

    @Override public Transform3d getCameraTransform() {
        return fixedCameraTransform;
    }

    /** Exposes the PhotonVision sim camera for {@code VisionSystemSim} wiring. */
    @Override
    public Optional<PhotonCameraSim> getCameraSim() {
        return Optional.of(cameraSim);
    }

    @Override
    public void drainResultQueue(List<ResultInterface> destination) {
        ResultInterface result;
        while ((result = resultQueue.poll()) != null) {
            destination.add(result);
            queueSize.decrementAndGet();
        }
    }

    @Override
    public List<ResultInterface> getResultQueue() {
        List<ResultInterface> results = new ArrayList<>();
        drainResultQueue(results);
        return results;
    }

    @Override public int getMaxQueueSize() {
        return maxQueueSize;
    }

    @Override public Notifier getNotifier() {
        return processingNotifier;
    }

    @Override public double getFrequency() {
        return processingFrequencyHz;
    }
    @Override public boolean isRunning() {
        return isRunning;
    }

    @Override
    public void setPipeline(int newPipelineIndex) {
        photonCamera.setPipelineIndex(newPipelineIndex);
    }

    @Override
    public void setCameraTransform(Transform3d newCameraTransform) {
        poseEstimator.setRobotToCameraTransform(newCameraTransform);
    }

    //#endregion

}
