package com.team1816.lib.subsystems;

import com.team1816.lib.BaseRobotState;
import com.team1816.lib.hardware.components.sensor.Camera;
import com.team1816.season.Robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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

import edu.wpi.first.apriltag.AprilTagFieldLayout;

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
    private VisionSystemSim visionSim;
    /**
     * The base standard deviations to use for non-multi-tag estimates
     */
    private final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
    /**
     * The base standard deviations to use for multi-tag estimates
     */
    private final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    /**
     * The standard deviations to use for an estimate that we should just throw out
     */
    private final Matrix<N3, N1> noTrustStdDevs = VecBuilder.fill(
        Double.NaN, Double.NaN, Double.NaN
    );

    /**
     * Field dimensions for bounds checking. Estimates outside the field are rejected.
     * Uses the AprilTag field layout to determine field size.
     */
    private final double fieldLengthMeters;
    private final double fieldWidthMeters;
    /** Margin outside the field boundary to still accept (accounts for robot size) */
    private static final double FIELD_BOUNDARY_MARGIN_METERS = 0.5;

    /**
     * Constructs a Vision subsystem.
     */
    public Vision() {
        cameras = factory.getCameras(NAME);
        aprilTagCameras = cameras.stream().filter(
            camera -> camera.detectionType == Camera.DetectionType.APRIL_TAG
        ).toList();

        // Get field dimensions from the AprilTag field layout
        fieldLengthMeters = aprilTagFieldLayout.getFieldLength();
        fieldWidthMeters = aprilTagFieldLayout.getFieldWidth();

        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("VisionSim");
            visionSim.addAprilTags(aprilTagFieldLayout);
            for (Camera camera : cameras) {
                camera.addToSim(visionSim);
            }
        }
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

        // The maximum distance a vision pose estimate can be from the current robot pose
        // estimate to allow the vision estimate to be used. Increased from 1.0m to 3.0m
        // to prevent a death spiral where pose drift causes all cameras to be permanently
        // ignored (old 1m gate was too tight — a small drift would reject all vision,
        // causing more drift, rejecting more vision, etc.)
        double visionEstimateDistanceThresholdMeters = 3.0;
        // The maximum difference the angle of a vision pose estimate can be from the angle
        // of the current robot pose estimate to allow the vision estimate to be used.
        double visionEstimateAngleThresholdRadians = Units.degreesToRadians(25.0);

        for (Camera camera : aprilTagCameras) {
            for (EstimatedRobotPose estimatedRobotPose : camera.getEstimatedRobotPosesFromAllUnreadResults()) {
                Pose2d visionEstimatedPose2d = estimatedRobotPose.estimatedPose.toPose2d();

                // Hard reject: if the estimated pose is outside the field boundaries
                // (with margin for robot size), the estimate is definitely wrong.
                if (!isWithinFieldBounds(visionEstimatedPose2d)) {
                    continue;
                }

                // Always apply distance and angle gates — even during pose loss.
                // During normal operation, the tight gate catches bad single-tag solves
                // (the main source of 3-4m outlier readings that are still on-field).
                // During pose loss, we widen the gate since odometry may have drifted,
                // but the agreement logic in BaseSuperstructure provides the real
                // filtering for that case.
                double distanceFromCurrentPose = visionEstimatedPose2d.getTranslation()
                    .getDistance(BaseRobotState.robotPose.getTranslation());
                double angleFromCurrentPose = Math.abs(
                    MathUtil.angleModulus(
                        visionEstimatedPose2d.getRotation()
                            .minus(BaseRobotState.robotPose.getRotation())
                            .getRadians()
                    )
                );

                // Use a wider gate during pose loss since odometry itself may have drifted
                double effectiveDistanceThreshold = BaseRobotState.hasAccuratePoseEstimate
                    ? visionEstimateDistanceThresholdMeters
                    : visionEstimateDistanceThresholdMeters * 3; // 9m — still rejects cross-field outliers
                double effectiveAngleThreshold = BaseRobotState.hasAccuratePoseEstimate
                    ? visionEstimateAngleThresholdRadians
                    : Math.PI; // Accept any angle during pose loss

                if (
                    distanceFromCurrentPose < effectiveDistanceThreshold
                        && angleFromCurrentPose < effectiveAngleThreshold
                ) {
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
            }
        }

        return posesWithStdDevs;
    }

    /**
     * Checks if a pose estimate is within the field boundaries (with margin).
     * Rejects any estimate that places the robot outside the field, which is
     * physically impossible and indicates a bad vision solve.
     */
    private boolean isWithinFieldBounds(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return x >= -FIELD_BOUNDARY_MARGIN_METERS
            && x <= fieldLengthMeters + FIELD_BOUNDARY_MARGIN_METERS
            && y >= -FIELD_BOUNDARY_MARGIN_METERS
            && y <= fieldWidthMeters + FIELD_BOUNDARY_MARGIN_METERS;
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
