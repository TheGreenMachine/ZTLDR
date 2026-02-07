package com.team1816.lib;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.List;

public final class BaseConstants {
    public static final class VisionConstants {
        public static final AprilTagFieldLayout aprilTagFieldLayout =
//            new AprilTagFieldLayout(
//                List.of(
//                    new AprilTag(15, new Pose3d(
//                        new Translation3d(1, 0, Units.inchesToMeters(24)),
//                        new Rotation3d(0, 0, Math.PI)
//                    ))
//                ), 16.48, 8.10
//            );
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
}
