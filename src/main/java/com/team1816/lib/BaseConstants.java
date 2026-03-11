package com.team1816.lib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class BaseConstants {
    public static final class VisionConstants {
        public static final AprilTagFieldLayout aprilTagFieldLayout =
//            new AprilTagFieldLayout(
//                List.of(
//                    new AprilTag(15, new Pose3d(
//                        new Translation3d(3, 3, Units.inchesToMeters(24)),
//                        new Rotation3d(0, 0, Math.PI)
//                    ))
//                ), 16.48, 8.10
//            );
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    public static final class DrivetrainConstants {
        /**
         * The default values to use as the state/odometry standard deviations for the drivetrain.
         * They represent the pose estimator's trust in the current state of the odometry estimate,
         * with higher values representing less trust. The state standard deviations actively being
         * used by the drivetrain can be set by using {@link
         * com.team1816.lib.subsystems.drivetrain.Swerve#setStateStdDevs(Matrix)}.
         */
        public static final Matrix<N3, N1> defaultStateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    }
}
