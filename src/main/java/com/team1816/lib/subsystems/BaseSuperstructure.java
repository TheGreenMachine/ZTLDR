package com.team1816.lib.subsystems;

import com.team1816.lib.subsystems.drivetrain.Swerve;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;

import java.util.List;

public abstract class BaseSuperstructure extends SubsystemBase {
    protected final Swerve swerve;
    protected final Vision vision;

    protected BaseSuperstructure(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
    }

    /**
     * Updates the drivetrain's pose estimate by passing in unread vision pose estimates from the
     * {@link Vision} subsystem. Vision filtering (single-tag heading check, ambiguity, distance)
     * is handled in Vision. The Kalman filter in the drivetrain blends vision measurements with
     * odometry based on the standard deviations provided with each measurement.
     */
    public void addVisionMeasurementsToDrivetrain() {
        List<Pair<EstimatedRobotPose, Matrix<N3, N1>>> visionMeasurements = vision
            .getVisionEstimatedPosesWithStdDevs();

        for (Pair<EstimatedRobotPose, Matrix<N3, N1>> visionMeasurement : visionMeasurements) {
            swerve.addVisionMeasurement(
                visionMeasurement.getFirst().estimatedPose.toPose2d(),
                visionMeasurement.getFirst().timestampSeconds,
                visionMeasurement.getSecond()
            );
        }
    }
}
