package com.team1816.lib.util.ShooterCalculator;

import com.team1816.lib.hardware.ShooterSettingsConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.List;

import static com.team1816.lib.Singleton.factory;

public class ShooterTableCalculator extends BaseShooterCalculator {

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

    public ShooterCalculatorResponse calculate(
        Translation3d shooter,
        Translation3d target,
        double angleOfEntryDegrees,
        boolean useChassisSpeedForHoodAngleAndSpeed,
        double lookAheadTimeSeconds
    ) {
        // Get the 2d distance from the shooter to the target.
        double distanceToTargetMeters = shooter.toTranslation2d().getDistance(target.toTranslation2d());
        double distanceToTargetInches = Units.metersToInches(distanceToTargetMeters);
        double inclineAngleRotations = getInclineAngleRotations(distanceToTargetInches);
        double inclineAngleDegrees = Units.rotationsToDegrees(inclineAngleRotations);
        double launchVelocityRPS = getLaunchVelocityRPS(distanceToTargetInches);

        return new ShooterCalculatorResponse(
            getTurretAngle(
                shooter.toTranslation2d(),
                target.toTranslation2d(),
                useChassisSpeedForHoodAngleAndSpeed
            ).getDegrees(),
            inclineAngleDegrees,
            launchVelocityRPS
        );
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
