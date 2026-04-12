package com.team1816.lib.subsystems.vision;

import com.team1816.lib.BaseConstants.VisionConstants;
import com.team1816.lib.util.GreenLogger;
import com.team1816.season.Robot;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import com.team1816.lib.subsystems.vision.cameras.ProcessorInterface;
import com.team1816.lib.subsystems.vision.filters.FilterInterface;
import com.team1816.lib.subsystems.vision.filters.PipelineFilter;
import com.team1816.lib.subsystems.vision.filters.ResultFilters;
import com.team1816.lib.subsystems.vision.processing.PoseFusionEngine;
import com.team1816.lib.subsystems.vision.results.ApriltagResult;
import com.team1816.lib.subsystems.vision.results.ResultInterface;
import com.team1816.lib.subsystems.vision.results.TrackedTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import java.util.*;

public class VisionSubsystem extends SubsystemBase {

    private final Map<String, ProcessorInterface> cameras;
    private final VisionConfiguration config;
    private final PoseFusionEngine fusionEngine;

    private final VisionSystemSim visionSystemSim;
    private final boolean isSimulation;

    private final PipelineFilter aprilTagFilter;
    private final VisionPoseConsumer poseConsumer;

    private final Swerve swerve;

    private final List<ResultInterface> combinedResults = new ArrayList<>(4);
    private final List<ApriltagResult> combinedApriltagResults = new ArrayList<>(4);

    private final Pose3d[] tagPoseScratch = new Pose3d[33];

    private volatile boolean hasValidPose;
    private ApriltagResult fusedResult = new ApriltagResult("", 0.0, 0.0, Pose2d.kZero,
        VecBuilder.fill(0.0, 0.0, 0.0), List.of(), 0.0, 0.0, 0.0, 0.0);
    private Pose2d fusedPose;

    public VisionSubsystem(
        Map<String, ProcessorInterface> cameras,
        AprilTagFieldLayout fieldLayout,
        VisionConfiguration configuration,
        VisionPoseConsumer poseConsumer,
        Swerve swerveSubsystem
    ) {
        this.cameras = cameras;
        this.fusionEngine = new PoseFusionEngine();
        this.visionSystemSim = new VisionSystemSim("main");
        this.poseConsumer = poseConsumer;
        this.config = configuration;
        this.swerve = swerveSubsystem;
        this.hasValidPose = false;

        this.aprilTagFilter = new PipelineFilter(buildFilterList(configuration, swerveSubsystem));

        isSimulation = Robot.isSimulation();

        if (isSimulation) {
            visionSystemSim.addAprilTags(fieldLayout);
            PhotonCamera.setVersionCheckEnabled(false);
        }

        for (ProcessorInterface camera : cameras.values()) {
            camera.start();

            if (isSimulation) {
                Optional<PhotonCameraSim> cameraSim = camera.getCameraSim();
                cameraSim.ifPresent(sim ->
                    visionSystemSim.addCamera(sim, camera.getCameraTransform())
                );
            }
        }

        setUpPeriodicLogging();
    }

    private static List<FilterInterface> buildFilterList(VisionConfiguration config, Swerve swerve) {
        List<FilterInterface> filters = new ArrayList<>();
//        filters.add(new ResultFilters.LatencyFilter(config.maxLatencyMs));
//        filters.add(new ResultFilters.AmbiguityFilter(config.maxAmbiguityScore));
//        filters.add(new ResultFilters.AreaFilter(config.minArea, config.maxArea));
//
//        if (config.maxOdometryDeviationMeters < Double.MAX_VALUE) {
//            filters.add(new ResultFilters.OdometryOutlierFilter(
//                swerve::getPose, config.maxOdometryDeviationMeters));
//        }

        return filters;
    }

    @Override
    public void periodic() {
        collectResults();
        filterAndCollectApriltags();

        if (!combinedApriltagResults.isEmpty()) {
            processApriltags();
        } else {
            hasValidPose = false;
        }

        if (isSimulation) {
            visionSystemSim.update(swerve.getPose());
        }
    }

    private void collectResults() {
        combinedResults.clear();
        for (ProcessorInterface entry : cameras.values()) {
            entry.drainResultQueue(combinedResults);
        }
    }

    private void filterAndCollectApriltags() {
        combinedApriltagResults.clear();
        for (int i = 0; i < combinedResults.size(); i++) {
            ResultInterface result = combinedResults.get(i);
            if (result instanceof ApriltagResult ar
                    && ar.getStdDevs() != null
                    && aprilTagFilter.test(ar)
            ) {
                combinedApriltagResults.add(ar);
            }
        }
    }

    private void processApriltags() {
        combinedApriltagResults.forEach(result -> poseConsumer.accept(result.getPose(),
            result.getTimestampSeconds(), result.getStdDevs()));
//
//        Optional<ApriltagResult> fusedResultOpt = fusionEngine.fusePoses(combinedApriltagResults, config);
//        if (fusedResultOpt.isEmpty()) {
//            hasValidPose = false;
//            return;
//        }
//
//        fusedResult = fusedResultOpt.get();
//        fusedPose = fusedResult.getPose();

//        if (swerve != null && swerve.isFlatDebounced()) {
//            poseConsumer.accept(
//                fusedPose,
//                fusedResult.getTimestampSeconds(),
//                fusedResult.getStdDevs()
//            );
//        }

        hasValidPose = true;
    }

    private void setUpPeriodicLogging() {
        String logPath = "VisionSubSystem/";

        GreenLogger.periodicLogList(logPath + "Combined Results", () -> combinedApriltagResults, ApriltagResult.class);
//        GreenLogger.periodicLog(logPath + "Vision Pose", () -> fusedResult.getPose(), Pose2d.struct);
//        GreenLogger.periodicLog(logPath + "Used Vision Pose", () -> swerve.getPastVisionPose(fusedResult.getTimestampSeconds()), Pose2d.struct);
//
//        GreenLogger.periodicLog(logPath + "XY Std Devs", () -> fusedResult.getStdDevs().get(0, 0));
//        GreenLogger.periodicLog(logPath + "Theta Std Devs", () -> fusedResult.getStdDevs().get(2, 0));
//
//        GreenLogger.periodicLog(logPath + "Avg Ambiguity", () -> fusedResult.getAmbiguity());
//        GreenLogger.periodicLog(logPath + "Avg Area", () -> fusedResult.getAverageArea());
//        GreenLogger.periodicLog(logPath + "Latency", () -> fusedResult.getLatencyMs());
//        GreenLogger.periodicLog(logPath + "Target Count", () -> fusedResult.getTargetCount());
//
//        GreenLogger.periodicLogList(logPath + "Tracked Apriltags", () -> getTargetPoses(fusedResult.getTrackedTags()),
//            Pose3d.class, Pose3d.struct);
//        GreenLogger.periodicLog(logPath + "Has Valid Pose", () -> hasValidPose);
    }

    /**
     * Builds a {@link Pose3d} array for the given tags using the field layout.
     */
    public List<Pose3d> getTargetPoses(List<TrackedTag> tags) {
        int count = 0;
        for (int i = 0; i < tags.size(); i++) {
            var tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(tags.get(i).fiducialId);
            if (tagPose.isPresent()) tagPoseScratch[count++] = tagPose.get();
        }

        Pose3d[] result = new Pose3d[count];
        System.arraycopy(tagPoseScratch, 0, result, 0, count);
        return Arrays.stream(result).toList();
    }

    public Pose2d getFusedPose() {
        if (hasValidPose == false) return null;
        return fusedPose;
    }

    public boolean hasValidPose() {
        return hasValidPose;
    }

    @FunctionalInterface
    public interface VisionPoseConsumer {
        void accept(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs);
    }

}
