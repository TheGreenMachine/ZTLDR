package com.team1816.lib.util;

import com.team1816.season.subsystems.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ShotLookup {
    private final PolynomialSplineFunction inclineAngleRotationsFunction, launchVelocityRPSFunction;
    private Shooter shooter;
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
    public double getLaunchAngleRadiansRPSExperiental(Translation2d translation) {
        var xDistance = translation.getX();
        var yDistance = translation.getY();
        // Gives a Launch angle in radians the enter angle is in degrees
        double launchAngle = Math.atan((2*yDistance/xDistance) - Math.tan((Math.PI/180) * (-45)));
        return launchAngle;
    }
    public double getLaunchVelocityRPSExperiental(double launchAngle,Translation2d translation) {
        var xDistance = translation.getX();
        var yDistance = translation.getY();
        // Everything is in Meters
        double launchVelocity = (xDistance/Math.cos(launchAngle)) * Math.sqrt((9.81)/(2*(xDistance*Math.tan(launchAngle)) - yDistance));
        return launchVelocity;
    }
}
