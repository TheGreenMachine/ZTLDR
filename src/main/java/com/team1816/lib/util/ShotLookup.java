package com.team1816.lib.util;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ShotLookup {
    private PolynomialSplineFunction angleInterpolator, powerInterpolator;

    public ShotLookup(double[] distances, double[] angles, double[] powers) {
        LinearInterpolator angleLI = new LinearInterpolator();
        LinearInterpolator powerLI = new LinearInterpolator();
        this.angleInterpolator = angleLI.interpolate(distances, angles);
        this.powerInterpolator = powerLI.interpolate(distances, powers);
    }

    public double getAngle(double distance) {
        if (angleInterpolator.isValidPoint(distance)) {
            return angleInterpolator.value(distance);
        }

        return 25; // some kind of default that needs to come from yaml - clark
    }

    public double getPower(double distance) {
        if (powerInterpolator.isValidPoint(distance)) {
            return powerInterpolator.value(distance);
        }

        return 25; // some kind of default that needs to come from yaml - clark
    }
}
