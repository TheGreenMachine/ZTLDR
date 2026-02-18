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
        var knots = angleInterpolator.getKnots();

        if (knots.length > 0) {
            if (distance < knots[0]) {
                // some low end default
                return 25; // TODO - Clark
            } else if (distance > knots[knots.length - 1]) {
                // some high end default
                return 25; // TODO - Clark
            } else {
                return angleInterpolator.value(distance);
            }
        }

        return 25; // // TODO - Clark some mid default as we don't have data points due to a bad load from yaml
    }

    public double getPower(double distance) {
        var knots = powerInterpolator.getKnots();

        if (knots.length > 0) {
            if (distance < knots[0]) {
                // some low end default
                return 25; // TODO - Clark
            } else if (distance > knots[knots.length - 1]) {
                // some high end default
                return 25; // TODO - Clark
            } else {
                return powerInterpolator.value(distance);
            }
        }

        return 25; // // TODO - Clark some mid default as we don't have data points due to a bad load from yaml
    }
}
