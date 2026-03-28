package com.team1816.lib.util;

import com.team1816.lib.hardware.ShooterSettingsConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.List;

import static com.team1816.lib.Singleton.factory;

public class ShooterTableCalculator extends BaseShooterCalculator {

    private final PolynomialSplineFunction inclineAngleRotationsFunction, launchVelocityRPSFunction;
    private double lookaheadTime = 1.0;

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

    public ShooterCalculatorResponse getShooterSettings(Pose2d robotPose, ChassisSpeeds groundSpeed, Translation2d target) {

        // Convert from robot-relative speed to field relative speed
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(groundSpeed, robotPose.getRotation());

        double predictedX = robotPose.getX() + (fieldSpeeds.vxMetersPerSecond * lookaheadTime);
        double predictedY = robotPose.getY() + (fieldSpeeds.vyMetersPerSecond * lookaheadTime);
        Translation2d predictedPose = new Translation2d(predictedX, predictedY);
        Translation2d robotToTargetVector = target.minus(predictedPose);
//
//        NetworkTable netTable = NetworkTableInstance.getDefault().getTable("");
//        DoubleArrayPublisher pub = netTable.getDoubleArrayTopic("Field/PredictedPose").publish();
//        pub.set(new double[] {
//            predictedX, predictedY, predictedPose.getAngle().getDegrees()
//        });

        double distanceToTargetMeters = robotToTargetVector.getNorm();
        double distanceToTargetInches = Units.metersToInches(distanceToTargetMeters);
        double inclineAngleRotations = getInclineAngleRotations(distanceToTargetInches);
        double inclineAngleDegrees = Units.rotationsToDegrees(inclineAngleRotations);
        double launchVelocityRPS = getLaunchVelocityRPS(distanceToTargetInches);
        Rotation2d targetHeading = target.minus(predictedPose).getAngle();
        return new ShooterCalculatorResponse(inclineAngleDegrees, launchVelocityRPS, targetHeading);
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
