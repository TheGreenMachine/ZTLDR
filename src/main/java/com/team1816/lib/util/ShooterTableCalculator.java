package com.team1816.lib.util;

import com.team1816.lib.hardware.ShooterSettingsConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.List;

import static com.team1816.lib.Singleton.factory;

public class ShooterTableCalculator implements IShooterCalculator {

    private final PolynomialSplineFunction inclineAngleRotationsFunction, launchVelocityRPSFunction;

    public ShooterTableCalculator() {
        ShooterSettingsConfig shooterSettings = factory.getShooterSettingsConfig();

        List<Double> distancesInches = shooterSettings.distancesInches;
        List<Double> inclineAnglesRotations = shooterSettings.inclineAnglesRotations;
        List<Double> launchVelocitiesRPS = shooterSettings.launchVelocitiesRPS;
        LinearInterpolator inclineAngleRotationsLI = new LinearInterpolator();
        LinearInterpolator launchVelocityRPSLI = new LinearInterpolator();

        double[] distancesInchesArray = distancesInches.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();
        double[] inclineAnglesRotationsArray = inclineAnglesRotations.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();
        double[] launchVelocitiesRPSArray = launchVelocitiesRPS.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();

        this.inclineAngleRotationsFunction = inclineAngleRotationsLI.interpolate(distancesInchesArray, inclineAnglesRotationsArray);
        this.launchVelocityRPSFunction = launchVelocityRPSLI.interpolate(distancesInchesArray, launchVelocitiesRPSArray);
    }

    public ShooterCalculatorResponse getShooterSettings(Translation2d shooter, Translation2d target, boolean useChassisSpeedForHoodAngleAndSpeed) {
        double distanceToTargetMeters = shooter.getDistance(target);
        double distanceToTargetInches = Units.metersToInches(distanceToTargetMeters);
        double inclineAngleRotations = getInclineAngleRotations(distanceToTargetInches);
        double inclineAngleDegrees = Units.rotationsToDegrees(inclineAngleRotations);
        double launchVelocityRPS = getLaunchVelocityRPS(distanceToTargetInches);

        return new ShooterCalculatorResponse(inclineAngleDegrees, launchVelocityRPS);
    }

    private double getInclineAngleRotations(double distanceInches) {
        var knots = inclineAngleRotationsFunction.getKnots();
        // Clamp the distance to within the interpolation range.
        double clampedDistanceInches = MathUtil.clamp(distanceInches, knots[0], knots[knots.length - 1]);
        return inclineAngleRotationsFunction.value(clampedDistanceInches);
    }

    private double getLaunchVelocityRPS(double distanceInches) {
        var knots = launchVelocityRPSFunction.getKnots();
        // Clamp the distance to within the interpolation range.
        double clampedDistanceInches = MathUtil.clamp(distanceInches, knots[0], knots[knots.length - 1]);
        return launchVelocityRPSFunction.value(clampedDistanceInches);
    }
}
