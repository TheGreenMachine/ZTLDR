package com.team1816.season.subsystems;

import com.team1816.season.RobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;


public class RobotPositionValues {
    public static Translation2d getDistanceToBlueHub(){
        Translation2d robotTranslation = RobotState.swerveDriveState.Pose.getTranslation();
        Translation2d hubTranslation = new Translation2d(4.6228,3.8608);
        Translation2d BlueDistance = robotTranslation.minus(hubTranslation);
        return BlueDistance;
    }
    public static Translation2d getDistanceToRedHub(){
        var transform = new Transform2d(RobotState.swerveDriveState.Pose, new Pose2d(11.915394,3.8608,Rotation2d.kZero));
        var RED_TRANSFORM = transform.getTranslation();
        return RED_TRANSFORM;
    }
    public static double getRedHypotonuse() {
        var Hypotonuse = getDistanceToRedHub().getNorm();
        return Hypotonuse;
    }
    // 11.915394,3.8608
    public static double getBlueHypotonuse(){
        Measure xDistance = getDistanceToBlueHub().getMeasureX();
        Measure yDistance = getDistanceToBlueHub().getMeasureY();
        double Blue_Hypotonuse = Math.sqrt(Math.pow(xDistance.magnitude(),2) + Math.pow(yDistance.magnitude(),2));
        return Blue_Hypotonuse;
    }
    public static double getBlueRatios(double Hypotonuse) {
         Measure xDistance = getDistanceToBlueHub().getMeasureX();
         double BlueRatio = (xDistance.magnitude()/Hypotonuse);
         return BlueRatio;
    }

    public static double getRedRatios() {
      var Hypotonuse = getDistanceToRedHub().getNorm();
      var xDistance = getDistanceToRedHub().getX();
        double RedRatio = (xDistance/Hypotonuse);
        return RedRatio;
    }




}
