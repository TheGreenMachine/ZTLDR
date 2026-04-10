package com.team1816.lib.util.ShooterCalculator;

import com.team1816.lib.hardware.LinearMPSToLauncherRPSConfig;
import edu.wpi.first.math.MathUtil;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.List;

import static com.team1816.lib.Singleton.factory;

public class LinearMPSToLauncherRPSLookup {

    private final PolynomialSplineFunction linearMPSToLauncherRPSFunction;

    public LinearMPSToLauncherRPSLookup() {
        // TODO: If we calibrate this using the linearMPSToLaunchRPS YAML, we can use the line below.
        //  For now we are just trying to use some physics to get data from the shooterSettings table.
//        LinearMPSToLauncherRPSConfig table = factory.getLinearMPSToLauncherRPSConfig();
        LinearMPSToLauncherRPSConfig table = factory.stealLinearMPSToLauncherRPSConfigFromShooterSettings(72);

        List<Double> linearVelocitiesMPS = table.linearVelocitiesMPS;
        List<Double> launchVelocitiesRPS = table.launchVelocitiesRPS;
        LinearInterpolator linearMPSToLauncherRPSLI = new LinearInterpolator();

        double[] linearVelocitiesMPSArray = linearVelocitiesMPS.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();
        double[] launchVelocitiesRPSArray = launchVelocitiesRPS.stream()
            .mapToDouble(Double::doubleValue)
            .toArray();

        this.linearMPSToLauncherRPSFunction = linearMPSToLauncherRPSLI.interpolate(linearVelocitiesMPSArray, launchVelocitiesRPSArray);
    }

    public double getLaunchVelocityRPS(double linearVelocityMPS) {
        var knots = linearMPSToLauncherRPSFunction.getKnots();
        // Clamp the distance to within the interpolation range.
        double clampedLinearVelocityMPS = MathUtil.clamp(linearVelocityMPS, knots[0], knots[knots.length - 1]);
        return linearMPSToLauncherRPSFunction.value(clampedLinearVelocityMPS);
    }
}
