package com.team1816.lib.util;

import com.team1816.lib.hardware.ShooterSettingsConfig;

import java.util.List;

import static com.team1816.lib.Singleton.factory;

public class ShooterTableCalculator {

    private final ShotLookup shotLookup;

    public ShooterTableCalculator() {
        ShooterSettingsConfig shooterSettings = factory.getShooterSettingsConfig();

        List<Double> distancesInches = shooterSettings.distancesInches;
        List<Double> inclineAnglesRotations = shooterSettings.inclineAnglesRotations;
        List<Double> launchVelocitiesRPS = shooterSettings.launchVelocitiesRPS;

        double[] distancesInchesArray = distancesInches.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();
        double[] inclineAnglesRotationsArray = inclineAnglesRotations.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();
        double[] launchVelocitiesRPSArray = launchVelocitiesRPS.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();

        shotLookup = new ShotLookup(distancesInchesArray, inclineAnglesRotationsArray, launchVelocitiesRPSArray);
    }

    public ShooterDistanceSetting getShooterDistanceSetting(double distanceInches) {
        return new ShooterDistanceSetting(
            shotLookup.getInclineAngleRotations(distanceInches),
            shotLookup.getLaunchVelocityRPS(distanceInches)
        );
    }

    public record ShooterDistanceSetting(double inclineAngleRotations, double launchVelocityRPS) {}
}
