package com.team1816.lib.subsystems;

import com.team1816.lib.BaseRobotState;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;

import java.util.List;

import static com.team1816.lib.BaseConstants.DrivetrainConstants.defaultStateStdDevs;

public abstract class BaseSuperstructure extends SubsystemBase {
    protected final Swerve swerve;
    protected final Vision vision;

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
     */
    public void addVisionMeasurementsToDrivetrain() {
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> visionMeasurements = vision
            .getVisionEstimatedPosesWithStdDevs();

        for (Pair<EstimatedRobotPose, Matrix<N3, N1>> visionMeasurement : visionMeasurements) {
            Matrix<N3, N1> visionStdDevs = visionMeasurement.getSecond();

            if (!BaseRobotState.hasAccuratePoseEstimate && goodVisionEstimatesSincePoseLoss == 0) {
                // If we just lost pose and haven't had any good vision estimates since, assume
                // very little trust in the current state estimate. This will essentially make the
                // next vision measurement added set the pose almost completely.
                swerve.setStateStdDevs(
                    VecBuilder.fill(100, 100, 100)
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
                final int minGoodPoseEstimatesUntilConfident = 10;

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
}
