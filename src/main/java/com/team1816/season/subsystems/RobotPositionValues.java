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
    public static double getBlueRatios() {
         Measure xDistance = getDistanceToBlueHub().getMeasureX();
         Measure yDistance = getDistanceToBlueHub().getMeasureY();
         double Hypotonuse = Math.sqrt(Math.pow(xDistance.magnitude(),2) + Math.pow(yDistance.magnitude(),2));
         double BlueRatio = (xDistance.magnitude()/Hypotonuse);
         return BlueRatio;
    }
    public static double getRedRatios() {
        Measure xDistance = getDistanceToRedHub().getMeasureX();
        Measure yDistance = getDistanceToRedHub().getMeasureY();
        double Hypotonuse = Math.sqrt(Math.pow(xDistance.magnitude(),2) + Math.pow(yDistance.magnitude(),2));
        double RedRatio = (xDistance.magnitude()/Hypotonuse);
        return RedRatio;
    }


}
