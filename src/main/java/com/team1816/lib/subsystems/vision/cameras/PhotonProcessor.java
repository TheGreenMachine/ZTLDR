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

/**
 * A processor backed by PhotonVision's camera and pose estimator.
 */
public class PhotonProcessor implements ProcessorInterface {

    private final String cameraName;
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d cameraTransform;
    private final PhotonPoseEstimator poseEstimator;
    private final StdDevCalculator stdDevCalculator;

    private final PhotonCameraSim cameraSim;

    private final ConcurrentLinkedQueue<ResultInterface> resultQueue;
    private final AtomicInteger queueSize;
    private final int maxQueueSize;

    private final Notifier processingNotifier;
    private final double processingFrequency;

    private volatile boolean isRunning;

    private final List<TrackedTag> tagScratch = new ArrayList<>(8);

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
        SimCameraProperties properties
    ) {
        this.cameraName = name;
        this.photonCamera = new PhotonCamera(cameraName);
        this.fieldLayout = layout;
        this.cameraTransform = transform;
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, cameraTransform);
        this.stdDevCalculator = calculator;

        this.cameraSim = new PhotonCameraSim(photonCamera, properties);

        this.resultQueue = new ConcurrentLinkedQueue<>();
        this.queueSize = new AtomicInteger(0);
        this.maxQueueSize = 4;

        this.processingNotifier  = new Notifier(this::process);
        this.processingFrequency = 20.0;
        this.processingNotifier.setName("Photon-" + cameraName);
        this.isRunning = false;

        setUpPeriodicLogging("VisionSubSystem/cameras/" + name + "/");
    }

    //#region Processing

    @Override
    public void start() {
        isRunning = true;
        processingNotifier.startPeriodic(1.0 / processingFrequency);
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

        for (PhotonPipelineResult rawResult : rawResults) {
            Optional<ApriltagResult> apriltagResult = processApriltagResult(rawResult);
            if (apriltagResult.isPresent()) {
                while (queueSize.get() >= maxQueueSize) {
                    if (resultQueue.poll() != null) queueSize.decrementAndGet();
                }
                resultQueue.add(apriltagResult.get());
                queueSize.incrementAndGet();
            }
        }
    }

    private Optional<ApriltagResult> processApriltagResult(PhotonPipelineResult rawResult) {
        if (!rawResult.hasTargets()) return Optional.empty();

        List<PhotonTrackedTarget> targets = rawResult.getTargets();
        seenAprilTagPoses = targets.stream()
            .map(target ->
                // Find the AprilTag's pose on the field based on its position relative to the
                // camera, the position of the camera on the robot, and the current estimated
                // position of the robot on the field.
                new Pose3d(BaseRobotState.robotPose)
                    .plus(cameraTransform)
                    .plus(target.getBestCameraToTarget())
            )
            .collect(Collectors.toList());
        seenAprilTagIDs = targets.stream()
            .map(PhotonTrackedTarget::getFiducialId)
            .collect(Collectors.toList());

        int targetCount = targets.size();

        double avgAmbiguity = calculateAmbiguity(targets);
        double avgArea = calculateAverageArea(targets);

        Optional<EstimatedRobotPose> poseOptional = (targetCount > 1)
            ? poseEstimator.estimateCoprocMultiTagPose(rawResult)
            : poseEstimator.estimateLowestAmbiguityPose(rawResult);

        if (poseOptional.isEmpty()) return Optional.empty();

        EstimatedRobotPose estimatedPose = poseOptional.get();
        Pose2d resultantPose = estimatedPose.estimatedPose.toPose2d();

        double distance = resultantPose.getTranslation().getDistance(
            BaseRobotState.robotPose.getTranslation());
        double angle = Math.abs(
            MathUtil.angleModulus(
                resultantPose.getRotation()
                    .minus(BaseRobotState.robotPose.getRotation())
                    .getRadians()
            ));
        double timestamp = Utils.fpgaToCurrentTime(estimatedPose.timestampSeconds);
        double latency = rawResult.metadata.getLatencyMillis();

        originalCalc = stdDevCalculator.calculate(
            avgAmbiguity,
            avgArea,
            latency,
            targetCount
        );

        greenCalc = stdDevCalculator.calculateEstimateStandardDeviations(estimatedPose);

        // Convert PhotonTrackedTarget -> TrackedTag (camera-agnostic).
        tagScratch.clear();
        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget target = targets.get(i);
            tagScratch.add(new TrackedTag(target.fiducialId, target.getArea(), target.getPoseAmbiguity()));
        }

        return Optional.of(new ApriltagResult(
            cameraName,
            timestamp,
            latency,
            resultantPose,
            greenCalc,
            tagScratch,
            avgAmbiguity,
            avgArea,
            distance,
            angle
        ));
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
            () -> new Pose3d(BaseRobotState.robotPose).plus(cameraTransform),
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

        double minAmbiguity = 0.15;
        for (PhotonTrackedTarget target : targets) {
            double a = target.getPoseAmbiguity();
            if (a >= 0 && a < minAmbiguity) minAmbiguity = a;
        }
        return minAmbiguity / Math.sqrt(targets.size());
    }

    private static double calculateAverageArea(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) return 0.0;
        double sum = 0.0;
        for (PhotonTrackedTarget target : targets) sum += target.getArea();
        return sum / targets.size();
    }

    //#endregion

    //#region ProcessorInterface

    @Override public String getCameraName() {
        return cameraName;
    }

    @Override public Transform3d getCameraTransform() {
        return cameraTransform;
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
        return processingFrequency;
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
