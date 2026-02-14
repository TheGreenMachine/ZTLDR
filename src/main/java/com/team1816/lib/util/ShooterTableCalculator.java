package com.team1816.lib.util;

import com.team1816.lib.hardware.ShooterSettingsConfig;

import java.util.List;

import static com.team1816.lib.Singleton.factory;

public class ShooterTableCalculator {

    ShotLookup shotLookup;

    public ShooterTableCalculator() {
        ShooterSettingsConfig shooterSettings = factory.getShooterSettingsConfig();

        List<Double> distances = shooterSettings.distances;
        List<Double> angles = shooterSettings.angles;
        List<Double> velocities = shooterSettings.powers;

        double[] distancesArray = distances.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();
        double[] anglesArray = angles.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();
        double[] velocitiesArray = velocities.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();

        shotLookup = new ShotLookup(distancesArray, anglesArray, velocitiesArray);
    }

    public ShooterDistanceSetting getShooterDistanceSetting(double distance) {
        return new ShooterDistanceSetting(shotLookup.getAngle(distance), shotLookup.getPower(distance));
    }

}
