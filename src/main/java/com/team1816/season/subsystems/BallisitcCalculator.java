package com.team1816.season.subsystems;
import com.team1816.lib.ballisticCalc.BallisticSolution;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
public class BallisitcCalculator {

// NOTE: Everything is in radians until the very end. I'll make a comment where radians are converted into degrees.
    /**
     * I know this isn't what it'll actually be structured, but this was the ultimate basis of what got need from the night of 1/03.
     * (I would recommend worrying about the initialAngle & initialVelocity later and just starting with start/end pose(will change as the robot moves) & distance)
     * There's some code in the .md file + our initial implementation of distance.
     */
    public class BallisticCalculator {
        private BallisitcConstants ballisticConstants;
        private BallisticContraints ballisticConstraints;
        private Translation3d missileTranslation;
        private Translation3d robotTranslation;
        private Translation2d robotSpeed;
        private Translation3d hubTranslation;

        public BallisticSolution getBallisticSolution(Translation3d launcher, Translation3d target, double velocity) {
            return new BallisticSolution(0, 0, 0);
        }

        public void getConstants(BallisitcConstants ballistic_Constants, BallisticContraints ballisitc_Constraints) {
            this.ballisticConstants = ballistic_Constants;
            this.ballisticConstraints = ballisitc_Constraints;
        }

        public Translation3d getRobotTranslation(Translation3d missileTranslation) {
            this.missileTranslation = missileTranslation;
            return this.missileTranslation;
        }

        public Translation3d getTargetTranslation(Translation3d hubTranslation) {
            this.hubTranslation = hubTranslation;
            return hubTranslation;
        }
        public double calculateHorizontalDistance(Translation3d missileTranslation, Translation3d targetStartTranslation ){
            var distance = Math.sqrt((missileTranslation.getX() - targetStartTranslation.getX()) * (missileTranslation.getX() - targetStartTranslation.getX())) + (((missileTranslation.getY() - targetStartTranslation.getY()) * (missileTranslation.getY() - targetStartTranslation.getY())));
            return distance;
        }
        public double calculateStationaryLaunchAngle(Translation3d missileTranslation, Translation3d targetStartTranslation) {
            var height = Math.abs(missileTranslation.getZ() - targetStartTranslation.getZ());
            var distance = calculateHorizontalDistance(missileTranslation,targetStartTranslation);
            var enterAngle = ballisticConstraints.getEnterAngle(60);
            var theta = (Math.atan(((2 * height) / distance)) - Math.tan(enterAngle));
            return theta;
        }

        public double calculateMissileVelocity(Translation3d missileTranslation, Translation3d targetStartTranslation) {
            var height = (targetStartTranslation.getZ() - missileTranslation.getZ());
            var distance = calculateHorizontalDistance(missileTranslation,targetStartTranslation);
            var launchAngle = calculateStationaryLaunchAngle(missileTranslation, targetStartTranslation);
            var gravity = ballisticConstants.getGravity();
            var missileVelocity = ((distance / Math.cos(launchAngle)) * Math.sqrt(gravity / (2 * (distance * Math.tan(launchAngle)) - height)));
            return missileVelocity;
        }

        public double getRotationAngle(Translation3d missileTranslation, Translation3d targetStartTranslation) {
            var distance = calculateHorizontalDistance(missileTranslation,targetStartTranslation);
            var xDistance = Math.abs(missileTranslation.getX() - hubTranslation.getX());
            var angle = (Math.acos(xDistance / distance));
            return angle;
        }

        public Translation2d findMovingLaunchAngle (Translation3d missileTranslation, Translation3d targetStartTranslation, Translation2d robotVelocity) {
            var angle = getRotationAngle(missileTranslation, targetStartTranslation);
            var velocity = calculateMissileVelocity(missileTranslation, targetStartTranslation);
            var translationVelocity = new Translation2d(Math.cos(angle) * velocity, Math.sin(angle) * velocity);
            var movingVelocity = translationVelocity.minus(robotVelocity);
            return movingVelocity;
        }


        public double actualAngle (Translation3d missileTranslation, Translation3d targetStartTranslation, Translation2d robotVelocity) {
            // Turns into degrees below
            var actualAngle = (findMovingLaunchAngle(missileTranslation, targetStartTranslation, robotVelocity).getAngle().getDegrees());
            return actualAngle;
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
}
