package com.team1816.lib.util;

import com.team1816.lib.Singleton;
import com.team1816.lib.hardware.factory.RobotFactory;

import java.util.HashMap;

public class ShooterTableCalculator {
    private final double RESOLUTION = 0.5;

    private RobotFactory robotFactory = Singleton.get(RobotFactory.class);
    private HashMap<Double, AngleVelocityPair> targetMap = robotFactory.getTargetConfig();

    public AngleVelocityPair getShooterSetting(double distance) {
        if (distance > 5 || distance < 0) {
            GreenLogger.log("Shooter distance is out of bounds");
            return null;
        }
        else if (targetMap.get(distance) == null) {
            double upperBound = roundToUpperBound(RESOLUTION, distance);
            double lowerBound = roundToLowerBound(RESOLUTION, distance);
            AngleVelocityPair upperPair = targetMap.get(upperBound);
            AngleVelocityPair lowerPair = targetMap.get(lowerBound);
            return new AngleVelocityPair((upperPair.getAngle()-lowerPair.getAngle())/RESOLUTION*(distance-lowerBound)+lowerPair.getAngle(), (upperPair.getPower()-lowerPair.getPower())/RESOLUTION*(distance-lowerBound)+lowerPair.getPower());
        }
        return targetMap.get(distance);
    }

    public double roundToLowerBound(double resolution, double value) {
        return value-value%resolution;
    }

    public double roundToUpperBound(double resolution, double value) {
        return value-value%resolution+resolution;
    }

}
