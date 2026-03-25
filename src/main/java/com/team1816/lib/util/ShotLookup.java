package com.team1816.lib.util;

import edu.wpi.first.math.MathUtil;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ShotLookup {
    private final PolynomialSplineFunction inclineAngleRotationsFunction, launchVelocityRPSFunction;

    public ShotLookup(double[] distancesInches, double[] inclineAnglesRotations, double[] launchVelocitiesRPS) {
        LinearInterpolator inclineAngleRotationsLI = new LinearInterpolator();
        LinearInterpolator launchVelocityRPSLI = new LinearInterpolator();
        this.inclineAngleRotationsFunction = inclineAngleRotationsLI.interpolate(distancesInches, inclineAnglesRotations);
        this.launchVelocityRPSFunction = launchVelocityRPSLI.interpolate(distancesInches, launchVelocitiesRPS);
    }

    public double getInclineAngleRotations(double distanceInches) {
        var knots = inclineAngleRotationsFunction.getKnots();
        // Clamp the distance to within the interpolation range.
        double clampedDistanceInches = MathUtil.clamp(distanceInches, knots[0], knots[knots.length - 1]);
        return inclineAngleRotationsFunction.value(clampedDistanceInches);
    }

    public double getLaunchVelocityRPS(double distanceInches) {
        var knots = launchVelocityRPSFunction.getKnots();
        // Clamp the distance to within the interpolation range.
        double clampedDistanceInches = MathUtil.clamp(distanceInches, knots[0], knots[knots.length - 1]);
        return launchVelocityRPSFunction.value(clampedDistanceInches);
    }
}
