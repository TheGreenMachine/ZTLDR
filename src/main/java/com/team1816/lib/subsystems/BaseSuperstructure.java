package com.team1816.lib.subsystems;

import com.team1816.lib.BaseRobotState;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;

import java.util.List;

public abstract class BaseSuperstructure extends SubsystemBase {
    protected final Swerve swerve;
    protected final Vision vision;

    /**
     * The maximum distance a vision pose estimate can be from the current robot pose estimate to
     * allow the vision estimate to be added to the pose estimate Kalman filter. This is to try to
     * filter out unreasonable vision results caused by <a href=
     * "https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html#ambiguity"
     * >pose ambiguity</a>.
     */
    protected final double visionEstimateDistanceThresholdMeters = 1.0;
    /**
     * The maximum difference the angle of a vision pose estimate can be from the angle of the
     * current robot pose estimate to allow the vision estimate to be added to the pose estimate
     * Kalman filter. This is to try to filter out unreasonable vision results caused by <a href=
     * "https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/3D-tracking.html#ambiguity"
     * >pose ambiguity</a>.
     */
    protected final double visionEstimateAngleThresholdRadians = Units.degreesToRadians(15.0);

    protected BaseSuperstructure(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
    }

    /**
     * Updates the drivetrain's pose estimate by passing in unread vision pose estimates from the
     * {@link Vision} subsystem.
     * <p>
     * This method first filters out vision estimates that are too far from the robot's current
     * pose estimate in position or rotation to avoid adding unreasonable estimates caused by
     * vision pose ambiguity. If the current pose estimate is expected to sometimes not be accurate
     * enough to use as a comparison to filter vision estimates (such as if bumps or other field
     * obstacles greatly throw off odometry), this method should be overridden in the season
     * implementation of the superstructure to deal with these season-specific cases.
     */
    public void addVisionMeasurementsToDrivetrain() {
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> visionMeasurements = vision
            .getVisionEstimatedPosesWithStdDevs();

        for (Pair<EstimatedRobotPose, Matrix<N3, N1>> visionMeasurement : visionMeasurements) {
            Pose2d visionEstimatedPose = visionMeasurement.getFirst().estimatedPose.toPose2d();

            // Only add the vision measurement if it is with the distance and angle thresholds from
            // the current pose estimate. This is to filter out unreasonable estimates caused by
            // pose ambiguity.
            if (
                // Check if the vision estimate is within the distance threshold of the current
                // pose estimate.
                visionEstimatedPose.getTranslation().getDistance(
                    BaseRobotState.robotPose.getTranslation()
                ) < visionEstimateDistanceThresholdMeters
                // Check if the vision estimate is within the angle threshold of the current pose
                // estimate. Get the absolute value of the difference between the angles
                // constrained from -pi radians to pi radians to find the positive shortest
                // difference.
                && Math.abs(
                    MathUtil.angleModulus(
                        visionEstimatedPose.getRotation()
                            .minus(BaseRobotState.robotPose.getRotation())
                            .getRadians()
                    )
                ) < visionEstimateAngleThresholdRadians
            ) {
                swerve.addVisionMeasurement(
                    visionEstimatedPose,
                    visionMeasurement.getFirst().timestampSeconds,
                    visionMeasurement.getSecond()
                );
            }
        }
    }
}
