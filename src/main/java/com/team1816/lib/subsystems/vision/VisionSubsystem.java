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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import java.util.*;

public class VisionSubsystem extends SubsystemBase {

    private final List<ProcessorInterface> cameras;
    private final VisionConfiguration config;
    private final PoseFusionEngine fusionEngine;

    private final VisionSystemSim visionSystemSim;
    private final boolean isSimulation;

    private final PipelineFilter resultFilter;
    private final List<ResultInterface> primaryRaw = new ArrayList<>(8);
    private final List<ApriltagResult> primaryFused = new ArrayList<>(8);

    private final VisionPoseConsumer poseConsumer;

    private final Swerve swerve;

    private final List<ResultInterface> combinedResults = new ArrayList<>(4);
    private final List<ApriltagResult> combinedApriltagResults = new ArrayList<>(4);

    private final Pose3d[] tagPoseScratch = new Pose3d[33];

    private volatile boolean hasValidPose;
    private Pose2d visionPose;

    public VisionSubsystem(
        List<ProcessorInterface> cameras,
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

        this.resultFilter = new PipelineFilter(buildFilterList(configuration, swerveSubsystem));

        isSimulation = Robot.isSimulation();

        if (isSimulation) {
            visionSystemSim.addAprilTags(fieldLayout);
            PhotonCamera.setVersionCheckEnabled(false);
        }

        for (ProcessorInterface camera : this.cameras) startCamera(camera);

        setUpPeriodicLogging();
    }

    private void startCamera(ProcessorInterface camera) {
        camera.start();
        if (isSimulation) {
            camera.getCameraSim().ifPresent(sim -> {
                visionSystemSim.addCamera(sim, camera.getCameraTransform());
                sim.enableDrawWireframe(false);
                sim.enableProcessedStream(false);
                sim.enableRawStream(false);
            });
        }
    }

    private static List<FilterInterface> buildFilterList(VisionConfiguration config, Swerve swerve) {
        List<FilterInterface> filters = new ArrayList<>();
        filters.add(new ResultFilters.LatencyFilter(config.maxLatencyMs));
        filters.add(new ResultFilters.AmbiguityFilter(config.maxAmbiguityScore));
        filters.add(new ResultFilters.AreaFilter(config.minArea, config.maxArea));

        if (config.maxOdometryDeviationMeters < Double.MAX_VALUE) {
            filters.add(new ResultFilters.OdometryOutlierFilter(
                swerve::getPose, config.maxOdometryDeviationMeters));
        }

        return filters;
    }

    @Override
    public void periodic() {
        double fpgaTimestamp = Timer.getFPGATimestamp();

        hasValidPose = processCameraPipeline(cameras, resultFilter, primaryRaw, primaryFused);

        publishDiagnostics(primaryFused);

        if (isSimulation && swerve != null) {
            visionSystemSim.update(swerve.getPose());
        }
    }

    private boolean processCameraPipeline(
        List<ProcessorInterface> cameras,
        PipelineFilter filter,
        List<ResultInterface> rawResults,
        List<ApriltagResult> collectedTagList
    ) {
        rawResults.clear();
        for (ProcessorInterface camera : cameras) camera.drainResultQueue(rawResults);

        collectedTagList.clear();
        for (ResultInterface result : rawResults) {
            if (result instanceof ApriltagResult tagResult
                && tagResult.getStdDevs() != null
                && filter.test(tagResult)) {
                collectedTagList.add(tagResult);
            }
        }

        if (collectedTagList.isEmpty()) return false;

        Optional<ApriltagResult> fused = fusionEngine.fusePoses(collectedTagList);
        if (fused.isEmpty()) return false;

        ApriltagResult result = fused.get();
        if (swerve != null && swerve.isFlatDebounced()) {
            for (ApriltagResult collectedResult : collectedTagList) {
            poseConsumer.accept(
                collectedResult.getPose(),
                collectedResult.getTimestampSeconds(),
                collectedResult.getStdDevs()
                );
            }

//            poseConsumer.accept(
//                result.getPose(),
//                result.getTimestampSeconds(),
//                result.getStdDevs()
//            );
        }
        return true;
    }

    private void publishDiagnostics(List<ApriltagResult> results) {
        if (results.isEmpty()) return;
        ApriltagResult latest = results.stream()
            .max((a, b) -> Double.compare(a.getTimestampSeconds(), b.getTimestampSeconds()))
            .orElse(results.get(0));

        visionPose = latest.getPose();
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
    public Pose3d[] getTargetPoses(List<TrackedTag> tags) {
        int count = 0;
        for (TrackedTag tag : tags) {
            Optional<Pose3d> pose = VisionConstants.aprilTagFieldLayout.getTagPose(tag.getFiducialId());
            if (pose.isPresent()) tagPoseScratch[count++] = pose.get();
        }
        Pose3d[] result = new Pose3d[count];
        System.arraycopy(tagPoseScratch, 0, result, 0, count);
        return result;
    }
    public Pose2d getVisionPose() {
        return hasValidPose ? visionPose : null;
    }

    public boolean hasAnyPose() {
        return hasValidPose;
    }


    @FunctionalInterface
    public interface VisionPoseConsumer {
        void accept(Pose2d robotPose, double timestamp, Matrix<N3, N1> stdDevs);
    }

}
