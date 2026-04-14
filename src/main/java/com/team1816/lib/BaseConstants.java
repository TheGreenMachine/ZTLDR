package com.team1816.lib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public final class BaseConstants {
    public static final class VisionConstants {
        // Use this to load the default AprilTag field layout for the season. This will probably
        // be the welded version of the field. This can also be used to load the AndyMark field or
        // fields from previous seasons by changing the kDefaultField to the desired field.
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.kDefaultField
        );

        // Use this to use a custom AprilTag field layout specified as a list of AprilTags with
        // their Pose3ds.
//        public static final AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(
//            List.of(
//                new AprilTag(15, new Pose3d(
//                    new Translation3d(3, 3, Units.inchesToMeters(24)),
//                    new Rotation3d(0, 0, Math.PI)
//                ))
//            ), 16.48, 8.10
//        );

        // Use this to load a custom AprilTag field layout from a JSON file in the deploy
        // directory. This can be useful for using a field layout calibrated with WPIcal.
//        public static final AprilTagFieldLayout aprilTagFieldLayout;
//        static {
//            AprilTagFieldLayout layout;
//            try {
//                Path layoutPath = Filesystem.getDeployDirectory().toPath().resolve("tag_layouts/testing-layout.json");
//                layout = new AprilTagFieldLayout(layoutPath);
//            } catch (IOException e) {
//                // If there was a problem loading the field layout (likely the file path was
//                // specified incorrectly), fall back to the default field.
//                layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
//                GreenLogger.log(e);
//                GreenLogger.log(
//                    "IOException while loading AprilTag field layout. See above stack trace for " +
//                        "details."
//                );
//            }
//            aprilTagFieldLayout = layout;
//        }
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
