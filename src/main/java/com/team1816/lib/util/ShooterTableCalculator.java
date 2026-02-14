package com.team1816.lib.util;

import com.team1816.lib.Singleton;
import com.team1816.lib.hardware.AngleVelocityPairConfig;
import com.team1816.lib.hardware.factory.RobotFactory;
import edu.wpi.first.math.Pair;

import java.util.HashMap;

public class ShooterTableCalculator {
    private final double RESOLUTION = 0.5;

    private final RobotFactory robotFactory = Singleton.get(RobotFactory.class);
    private final HashMap<String, AngleVelocityPairConfig> targetMap = robotFactory.getTargetConfig();

    public Pair<Double, Double> getShooterSetting(double distance) {
        if (distance > 5 || distance < 0) {
            GreenLogger.log("Shooter distance is out of bounds");
            return null;
        } else if (targetMap.get(String.valueOf(distance)) == null) {
            double upperBound = roundToUpperBound(RESOLUTION, distance);
            double lowerBound = roundToLowerBound(RESOLUTION, distance);
            double upperAngle = targetMap.get(String.valueOf(upperBound)).angle;
            double upperVelocity = targetMap.get(String.valueOf(upperBound)).velocity;
            double lowerAngle = targetMap.get(String.valueOf(lowerBound)).angle;
            double lowerVelocity = targetMap.get(String.valueOf(lowerBound)).velocity;
            double returnAngle = (upperAngle - lowerAngle) / RESOLUTION * (distance - lowerBound) + lowerAngle;
            double returnVelocity = (upperVelocity - lowerVelocity) / RESOLUTION * (distance - lowerBound) + lowerVelocity;
            return Pair.of(returnAngle, returnVelocity);
        }
        return Pair.of(targetMap.get(String.valueOf(distance)).angle, targetMap.get(String.valueOf(distance)).velocity);
    }

    public double roundToLowerBound(double resolution, double value) {
        return value - value % resolution;
    }

    public double roundToUpperBound(double resolution, double value) {
        return value - value % resolution + resolution;
    }

}
