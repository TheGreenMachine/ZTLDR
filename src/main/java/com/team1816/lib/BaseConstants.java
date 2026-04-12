package com.team1816.lib;

import com.team1816.lib.subsystems.vision.cameras.PhotonProcessor;
import com.team1816.lib.subsystems.vision.cameras.ProcessorInterface;
import com.team1816.lib.subsystems.vision.processing.StdDevCalculator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.photonvision.simulation.SimCameraProperties;

import java.util.Map;

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

        public static final SimCameraProperties simCameraProperties = new SimCameraProperties();
        static {
            simCameraProperties.setCalibration(1600, 1200, Rotation2d.fromDegrees(97.65));
            simCameraProperties.setCalibError(0.05, 0.005);
            simCameraProperties.setFPS(50);
            simCameraProperties.setAvgLatencyMs(20);
            simCameraProperties.setLatencyStdDevMs(0);
        }

        // TODO: Tune these values more (PLEASE!)
        public static final class StdDevConstants {
            public static final double baseXYStdDev = 0.42;  // 0.18
            public static final double baseThetaStdDev = 0.87;  // 0.5 — heading from vision is still less reliable
            public static final double ambiguityWeight = 0.3;  // 0.6
            public static final double areaWeight = 0.8;  // 0.6
            public static final double latencyWeight = 0.9;  // 0.4
        }

        public static final class CameraConstants {

            /** Currently the transform of the evan camera */
            public static final Transform3d Forward_Left = new Transform3d(
                new Translation3d(
                    -12.66,
                    12.1,
                    18
                ),
                new Rotation3d(
                    Math.toRadians(0),
                    Math.toRadians(-8),
                    Math.toRadians(55)
                )
            );

            /** Currently the transform of the daniil camera */
            public static final Transform3d Forward_Right =  new Transform3d(
                new Translation3d(
                    -12.66,
                    -12.1,
                    18
                ),
                new Rotation3d(
                    Math.toRadians(0),
                    Math.toRadians(-8),
                    Math.toRadians(-55)
                )
            );

            /** Currently the transform of the val camera */
            public static final Transform3d Backward_Left =  new Transform3d(
                new Translation3d(
                    -12.66,
                    12.1,
                    16
                ),
                new Rotation3d(
                    Math.toRadians(0),
                    Math.toRadians(-8),
                    Math.toRadians(155)
                )
            );

            /** Currently the transform of the gollum camera */
            public static final Transform3d Backward_Right =  new Transform3d(
                new Translation3d(
                    -12.66,
                    -12.1,
                    16
                ),
                new Rotation3d(
                    Math.toRadians(0),
                    Math.toRadians(-8),
                    Math.toRadians(-155)
                )
            );

            //-----------------------------------------------------

            public static final Map<String, ProcessorInterface> cameras = Map.of(
                "fl", new PhotonProcessor(
                    "Forward_Left",
                    aprilTagFieldLayout,
                    Forward_Left,
                    new StdDevCalculator(1.3),
                    simCameraProperties
                ),
                "fr", new PhotonProcessor(
                    "Forward_Right",
                    aprilTagFieldLayout,
                    Forward_Right,
                    new StdDevCalculator(1.3),
                    simCameraProperties
                ),
                "bl", new PhotonProcessor(
                    "Backward_Left",
                    aprilTagFieldLayout,
                    Backward_Left,
                    new StdDevCalculator(1.3),
                    simCameraProperties
                ),
                "br", new PhotonProcessor(
                    "Backward_Right",
                    aprilTagFieldLayout,
                    Backward_Right,
                    new StdDevCalculator(1.3),
                    simCameraProperties
                )
            );
        }
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
