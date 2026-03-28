package com.team1816.lib.util;

import com.team1816.lib.BaseRobotState;
import com.team1816.season.subsystems.Shooter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import java.util.function.Function;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;


public class ShotLookup {
    BaseRobotState baseRobotState;
    private final double[] exitVelocity;
    private final List<Function<Double, Double>> rpsFunctions = new ArrayList<>();
//    private final PolynomialSplineFunction inclineAngleRotationsFunction, launchVelocityRPSFunction;
    private Shooter shooter;
    public ShotLookup(double[] exitVelocity, String[] launchVelocitiesRPS) {
        this.exitVelocity = exitVelocity;
        // Initialize the storage list
        //this.rpsFunctions = new ArrayList<>();

        Pattern pattern = Pattern.compile("([-+]?\\d*\\.?\\d*)x(?:\\^(\\d+))?|([-+]?\\d*\\.?\\d+)");
        if (rpsFunctions.isEmpty()){
            GreenLogger.log("nothing in rpsFunctions");
        }
        for (String expression : launchVelocitiesRPS) {
            final Map<Integer, Double> polynomialMap = new HashMap<>();
            String clean = expression.replaceAll("\\s+", "").replace("-", "+-").replace("++", "+");
            Matcher matcher = pattern.matcher(clean);

            while (matcher.find()) {
                String coeffStr = matcher.group(1);
                String expStr = matcher.group(2);
                String constStr = matcher.group(3);

                if (constStr != null && !constStr.isEmpty() && !constStr.equals("+")) {
                    double val = Double.parseDouble(constStr);
                    polynomialMap.put(0, polynomialMap.getOrDefault(0, 0.0) + val);
                } else if (coeffStr != null) {
                    double coeff;
                    if (coeffStr.isEmpty() || coeffStr.equals("+")) coeff = 1.0;
                    else if (coeffStr.equals("-")) coeff = -1.0;
                    else coeff = Double.parseDouble(coeffStr);

                    int exponent = (expStr == null) ? 1 : Integer.parseInt(expStr);
                    polynomialMap.put(exponent, polynomialMap.getOrDefault(exponent, 0.0) + coeff);
                }
            }

            // The 'lambda' (the actual function) is added to the rpsFunctions list here
            this.rpsFunctions.add((x) -> {
                double result = 0;
                for (Map.Entry<Integer, Double> term : polynomialMap.entrySet()) {
                    result += term.getValue() * Math.pow(x, term.getKey());
                }
                return result;
            });
        }
    }

    // You can now access them anywhere else in this class like this:
    public double getRPS(int index, double x) {
        // 1. Grab the function from the list at the specific index
        Function<Double, Double> equation = rpsFunctions.get(index);

        // 2. Plug in 'x' and return the result
        return equation.apply(x);
    }

//    public double getInclineAngleRotations(double distanceInches) {
//        var knots = inclineAngleRotationsFunction.getKnots();
//        // Clamp the distance to within the interpolation range.
//        double clampedDistanceInches = MathUtil.clamp(distanceInches, knots[0], knots[knots.length - 1]);
//        return inclineAngleRotationsFunction.value(clampedDistanceInches);
//    }
//
//    public double getLaunchVelocityRPS(double distanceInches) {
//        var knots = launchVelocityRPSFunction.getKnots();
//        // Clamp the distance to within the interpolation range.
//        double clampedDistanceInches = MathUtil.clamp(distanceInches, knots[0], knots[knots.length - 1]);
//        return launchVelocityRPSFunction.value(clampedDistanceInches);
//    }
    public double getLaunchAngleRadiansRPSExperiental(Translation2d translation) {
        // TODO: add gearing for the motors
        double launchAngleGearing = 0;
        var xDistance = translation.getX();
        var yDistance = translation.getY();
        // Gives a Launch angle in radians the enter angle is in degrees
        double launchAngle = Math.atan((2*yDistance/xDistance) - Math.tan((Math.PI/180) * (-45)));
        double motorLaunchAngle = launchAngle * launchAngleGearing;
        return motorLaunchAngle;
    }
    public double getLaunchVelocityRPSExperiental(double launchAngle,Translation2d translation) {
        var xDistance = translation.getX();
        var yDistance = translation.getY();
        int index = 0;
        int formulaIndex = 0;
        // Everything is in Meters
        double Velocity = (xDistance/Math.cos(launchAngle)) * Math.sqrt((9.81)/(2*(xDistance*Math.tan(launchAngle)) - yDistance));
        for (double velocity:exitVelocity) {
            if (exitVelocity[index]>velocity){
                formulaIndex = index;
            }
            index = index + 1;
        }
        double launchVelocityStatic = getRPS(formulaIndex,Velocity);
        ChassisSpeeds robotSpeed = BaseRobotState.robotSpeeds;
        double velocityX = robotSpeed.vxMetersPerSecond;
        double velocityY = robotSpeed.vyMetersPerSecond;
        // TODO: Not sure if my moving vector subtraction is correct
        double velocity = Math.sqrt((velocityX * velocityX) + (velocityY*velocityY));
        double launchVelocityRPS = launchVelocityStatic - velocity;
        return launchVelocityRPS;
    }
}
