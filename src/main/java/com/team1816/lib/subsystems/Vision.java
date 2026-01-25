package com.team1816.lib.subsystems;

import com.team1816.lib.hardware.components.sensor.Camera;
import com.team1816.season.Robot;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

/**
 * This subsystem handles reading and interpreting data from cameras and simulating camera
 * interactions. Currently, we support using cameras for reading <a href=
 * "https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html"
 * >AprilTags</a> to correct our estimate of the robot's position on the field.
 * <p>
 * We use <a href="https://docs.photonvision.org/">PhotonVision</a> for vision processing.
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
    private final VisionSystemSim visionSim;
    /**
     * The base standard deviations to use for non-multi-tag estimates
     */
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
    /**
     * The base standard deviations to use for multi-tag estimates
     */
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    /**
     * Constructs a Vision subsystem.
     */
    public Vision() {
        cameras = factory.getCameras(NAME);
        aprilTagCameras = cameras.stream().filter(
            camera -> camera.detectionType == Camera.DetectionType.APRIL_TAG
        ).toList();

        visionSim = new VisionSystemSim("VisionSim");
        if (Robot.isSimulation()) {
            visionSim.addAprilTags(aprilTagFieldLayout);
            for (Camera camera : cameras) {
                camera.addToSim(visionSim);
            }
        }
    }

    /**
     * Gets all {@link EstimatedRobotPose}s that have not yet been returned by calls to this method
     * from all AprilTag cameras. Estimates are in {@link Pair}s with the standard deviations for
     * the estimate calculated by {@link #calculateEstimationStandardDeviations(EstimatedRobotPose)
     * }.
     *
     * @return All unread vision pose estimates with corresponding standard deviations.
     */
    public List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> getVisionEstimatedPosesWithStdDevs() {
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> posesWithStdDevs = new ArrayList<>();
        for (Camera camera : aprilTagCameras) {
            for (EstimatedRobotPose estimatedRobotPose : camera.getEstimatedRobotPosesFromAllUnreadResults()) {
                Matrix<N3, N1> standardDeviations = calculateEstimationStandardDeviations(estimatedRobotPose);
                posesWithStdDevs.add(Pair.of(estimatedRobotPose, standardDeviations));
            }
        }
        return posesWithStdDevs;
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
     * CTRE recommends typically only adding vision measurements to your estimator if they are
     * within one meter or so of the current pose estimate to eliminate unreasonable results caused
     * by <a href="https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html#ambiguity">
     * pose ambiguity</a>. Measurements with rotations significantly off from the current estimate
     * should probably also be thrown out. This method intentionally does <i>not</i> handle
     * rejecting estimates in these cases to allow for cases where the current estimate may not be
     * reasonable enough to compare to, so this should be handled in season specific code.
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
     * @return The calculated standard deviations for the vision pose estimation (x position in
     * meters, y position in meters, and heading in radians, although the units are somewhat
     * ambiguous).
     */
    public Matrix<N3, N1> calculateEstimationStandardDeviations(EstimatedRobotPose estimatedRobotPose) {
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
                // layout. It couldn't have given us an estimation unless there was at least one
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
            if (closestDistance > 4 || lowestAmbiguity > 0.2) {
                return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
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

    @Override
    public void simulationPeriodic() {
        visionSim.update(new Pose2d(0.5, 4, Rotation2d.kZero)); //TODO: make this use the raw odometry pose/"actual" sim pose
    }

    @Override
    public void periodic() {
        getVisionEstimatedPosesWithStdDevs();
    }
}
