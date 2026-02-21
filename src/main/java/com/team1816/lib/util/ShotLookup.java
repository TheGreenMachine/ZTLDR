package com.team1816.lib.util;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ShotLookup {
    private PolynomialSplineFunction angleInterpolator, powerInterpolator;
    private double minimumDefaultAngle, minimumDefaultPower, mediumDefaultAngle, mediumDefaultPower, maximumDefaultAngle, maximumDefaultPower;

    public ShotLookup(double[] distances, double[] angles, double[] powers, double minimumDefaultAngle, double minimumDefaultPower, double mediumDefaultAngle, double mediumDefaultPower, double maximumDefaultAngle, double maximumDefaultPower) {
        LinearInterpolator angleLI = new LinearInterpolator();
        LinearInterpolator powerLI = new LinearInterpolator();
        this.angleInterpolator = angleLI.interpolate(distances, angles);
        this.powerInterpolator = powerLI.interpolate(distances, powers);
        this.minimumDefaultAngle = minimumDefaultAngle;
        this.minimumDefaultPower = minimumDefaultPower;
        this.mediumDefaultAngle = mediumDefaultAngle;
        this.mediumDefaultPower = mediumDefaultPower;
        this.maximumDefaultAngle = maximumDefaultAngle;
        this.maximumDefaultPower = maximumDefaultPower;
    }

    public double getAngle(double distance) {
        var knots = angleInterpolator.getKnots();

        if (knots.length > 0) {
            if (distance < knots[0]) {
                return minimumDefaultAngle;
            } else if (distance > knots[knots.length - 1]) {
                return maximumDefaultAngle;
            } else {
                return angleInterpolator.value(distance);
            }
        }

        return mediumDefaultAngle;
    }

    public double getPower(double distance) {
        var knots = powerInterpolator.getKnots();

        if (knots.length > 0) {
            if (distance < knots[0]) {
                return minimumDefaultPower;
            } else if (distance > knots[knots.length - 1]) {
                return maximumDefaultPower;
            } else {
                return powerInterpolator.value(distance);
            }
        }

        return mediumDefaultPower;
    }
}
