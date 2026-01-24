package com.team1816.season.subsystems;

import com.team1816.season.RobotState;
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
        Translation2d robotTranslation = RobotState.swerveDriveState.Pose.getTranslation();
        Translation2d hubTranslation = new Translation2d(11.915394,3.8608);
        Translation2d RedDistance = robotTranslation.minus(hubTranslation);
        return RedDistance;
    }
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
    public static double getRedHypotonuse(){
        Measure xDistance = getDistanceToRedHub().getMeasureX();
        Measure yDistance = getDistanceToRedHub().getMeasureY();
        double Red_Hypotonuse = Math.sqrt(Math.pow(xDistance.magnitude(),2) + Math.pow(yDistance.magnitude(),2));
        return Red_Hypotonuse;
    }
    public static double getRedRatios(double Hypotonuse) {
        Measure xDistance = getDistanceToRedHub().getMeasureX();
        double RedRatio = (xDistance.magnitude()/Hypotonuse);
        return RedRatio;
    }




}
