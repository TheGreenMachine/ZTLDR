package com.team1816.lib.util;

import com.team1816.lib.hardware.ShooterSettingsConfig;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

import static com.team1816.lib.Singleton.factory;

public class ShooterTableCalculator {

    private final ShotLookup shotLookup;

    public ShooterTableCalculator() {
        ShooterSettingsConfig shooterSettings = factory.getShooterSettingsConfig();
        List<Double> exitVelocity = shooterSettings.exitVelocity;
        List<String> launchVelocitiesRPS = shooterSettings.launchVelocitiesRPS;

        double[] exitVelocityArray = exitVelocity.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();
        String[] launchVelocitiesRPSArray = launchVelocitiesRPS.stream()
            .map(String::valueOf)
            .toArray(String[]::new);

        shotLookup = new ShotLookup(exitVelocityArray, launchVelocitiesRPSArray);
    }

    public ShooterDistanceSetting getShooterDistanceSetting(Translation2d translation) {
        return new ShooterDistanceSetting(
            shotLookup.getLaunchAngleRadiansRPSExperiental(translation),
            shotLookup.getLaunchVelocityRPSExperiental(shotLookup.getLaunchAngleRadiansRPSExperiental(translation),translation, shotLookup.getXVelocity(shotLookup.getLaunchAngleRadiansRPSExperiental(translation),translation),shotLookup.getYVelocity(shotLookup.getLaunchAngleRadiansRPSExperiental(translation),translation))
        );

    }

    public record ShooterDistanceSetting(double inclineAngleRotations, double launchVelocityRPS) {}
}
