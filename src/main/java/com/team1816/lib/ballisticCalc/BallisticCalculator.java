package com.team1816.lib.ballisticCalc;

import com.team1816.season.RobotContainer;
import com.team1816.season.RobotState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * I know this isn't what it'll actually be structured, but this was the ultimate basis of what got need from the night of 1/03.
 * (I would recommend worrying about the initialAngle & initialVelocity later and just starting with start/end pose(will change as the robot moves) & distance)
 * There's some code in the .md file + our initial implementation of distance.
 */
public class BallisticCalculator {
    private ballisticConstants ballistic_Constants;
    private BallisticConstraints ballisitc_Constraints;
    private Translation3d MissleTranslation;
    private Translation3d RobotTranslation;
    private Translation2d RobotSpeed;
    private Translation3d hubTranslation;

    public void getConstants(ballisticConstants ballistic_Constants, BallisticConstraints ballisitc_Constraints) {
        this.ballistic_Constants = ballistic_Constants;
        this.ballisitc_Constraints = ballisitc_Constraints;
    }

    public Translation3d getRobotTranslation(Translation3d missileTranslation) {
        this.MissleTranslation = missileTranslation;
        return MissleTranslation;
    }

    public Translation3d getTargetTranslation(Translation3d hubTranslation) {
        this.hubTranslation = hubTranslation;
        return hubTranslation;
    }

    public double calculateLaunchAngle(Translation3d missileTranslation, Translation3d targetStartTranslation) {
        var height = Math.abs(missileTranslation.getZ() - targetStartTranslation.getZ());
        var distance = Math.sqrt((missileTranslation.getX() - targetStartTranslation.getX()) * missileTranslation.getX() - targetStartTranslation.getX()) + ((missileTranslation.getY() - targetStartTranslation.getY() * missileTranslation.getY() - targetStartTranslation.getY()));
        var enterAngle = ballisitc_Constraints.getEnterAngle(60);
        var Theta = Math.toDegrees(Math.atan(((2 * height) / distance)) - Math.tan(enterAngle));
        return Theta;
    }

    public double calculateMissleVelocity(Translation3d missileTranslation, Translation3d targetStartTranslation) {
        var height = Math.abs(missileTranslation.getZ() - targetStartTranslation.getZ());
        var distance = Math.sqrt((missileTranslation.getX() - targetStartTranslation.getX()) * missileTranslation.getX() - targetStartTranslation.getX()) + ((missileTranslation.getY() - targetStartTranslation.getY() * missileTranslation.getY() - targetStartTranslation.getY()));
        var launchAngle = calculateLaunchAngle(missileTranslation, targetStartTranslation);
        var Gravity = ballistic_Constants.getGravity(9.81);
        var missleVelocity = ((distance / Math.cos(launchAngle)) * Math.sqrt(Gravity / (2 * (distance * Math.tan(launchAngle)) - height)));
        return missleVelocity;
    }

    public double getRotationAngle(Translation3d missileTranslation, Translation3d targetStartTranslation) {
        var distance = Math.sqrt((missileTranslation.getX() - targetStartTranslation.getX()) * missileTranslation.getX() - targetStartTranslation.getX()) + ((missileTranslation.getY() - targetStartTranslation.getY() * missileTranslation.getY() - targetStartTranslation.getY()));
        var xDistance = Math.abs(missileTranslation.getX() - hubTranslation.getX());
        var angle = Math.toDegrees(Math.acos(xDistance / distance));
        return angle;
    }

    public Translation2d findACTUALLaunchAngle(Translation3d missileTranslation, Translation3d targetStartTranslation, Translation2d robotVelocity) {
        var angle = getRotationAngle(missileTranslation, targetStartTranslation);
        var velocity = calculateMissleVelocity(missileTranslation, targetStartTranslation);
        var translationVelocity = new Translation2d(Math.cos(angle) * velocity, Math.sin(angle) * velocity);
        var ACTUALLVelocity = translationVelocity.minus(robotVelocity);
        return ACTUALLVelocity;
    }

    public double actuallAngle(Translation3d missileTranslation, Translation3d targetStartTranslation, Translation2d robotVelocity) {
        var ACTUALLAngle = (findACTUALLaunchAngle(missileTranslation, targetStartTranslation, robotVelocity).getAngle().getDegrees());
        return ACTUALLAngle;
    }


    public double getEndPose(double pose) {
        return pose;
    }

    public double getLaunchAngle(double launchAngle) {
        return launchAngle;
    }

    public double getInitialVelocity(double initialVelocity) {
        return initialVelocity;
    }


}
