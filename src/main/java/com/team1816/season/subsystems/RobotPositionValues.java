package com.team1816.season.subsystems;

import com.team1816.season.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;


public class RobotPositionValues {
    //    public static Translation2d getDistanceToBlueHub(){
//        var transform = new Transform2d(RobotState.swerveDriveState.Pose, new Pose2d(4.6228,3.8608,Rotation2d.kZero));
//        var BlueDistance = transform.getTranslation();
//        return BlueDistance;
//    }
//    public static Translation2d getDistanceToRedHub(){
//        var transform = new Transform2d(RobotState.swerveDriveState.Pose, new Pose2d(11.915394,3.8608,Rotation2d.kZero));
//        var RED_TRANSFORM = transform.getTranslation();
//        return RED_TRANSFORM;
//    }
    public static Translation2d getDistance() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            var transform = new Transform2d(RobotState.swerveDriveState.Pose, new Pose2d(4.6228, 3.8608, Rotation2d.kZero));
            var BlueDistance = transform.getTranslation();
            return BlueDistance;
        } else {
            var transform = new Transform2d(RobotState.swerveDriveState.Pose, new Pose2d(11.915394, 3.8608, Rotation2d.kZero));
            var RED_TRANSFORM = transform.getTranslation();
            return RED_TRANSFORM;
        }
    }

    public static double getHypotonuse() {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            var Hypotonuse = getDistance().getNorm();
            return Hypotonuse;
        } else {
            var transform = new Transform2d(RobotState.swerveDriveState.Pose, new Pose2d(11.915394, 3.8608, Rotation2d.kZero));
            var RED_TRANSFORM = transform.getTranslation();
            var Hypotonuse = RED_TRANSFORM.getNorm();
            return Hypotonuse;
        }

    }

    public static double getBlueRatios() {
        var Hypotonuse = getHypotonuse();
        var xDistance = getDistance().getX();
        double BlueRatio = (xDistance / Hypotonuse);
        return BlueRatio;
    }

    public static double getRedRatios() {
        var Hypotonuse = getHypotonuse();
        var xDistance = getDistance().getX();
        double RedRatio = (xDistance / Hypotonuse);
        return RedRatio;
    }
}
