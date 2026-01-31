package com.team1816.lib.util.ballisticCalc.src.main.java.com.ballistic;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Represents a complete solution for launching a projectile to hit a target.
 */
public class LaunchSolution {
    private final boolean solutionFound;
    private final double launchVelocity;        // m/s (speed relative to launcher)
    private final double pitchAngle;            // radians (elevation angle)
    private final double yawAngle;              // radians (horizontal angle, world frame)
    private final double flightTime;            // seconds
    private final Translation3d launchVelocityVector; // Complete velocity vector (world frame)
    private final Translation3d predictedLandingPosition;
    private final double landingAngle;          // radians (angle of descent)
    private final String failureReason;

    private LaunchSolution(boolean solutionFound, double launchVelocity, double pitchAngle,
                           double yawAngle, double flightTime, Translation3d launchVelocityVector,
                           Translation3d predictedLandingPosition, double landingAngle, String failureReason) {
        this.solutionFound = solutionFound;
        this.launchVelocity = launchVelocity;
        this.pitchAngle = pitchAngle;
        this.yawAngle = yawAngle;
        this.flightTime = flightTime;
        this.launchVelocityVector = launchVelocityVector;
        this.predictedLandingPosition = predictedLandingPosition;
        this.landingAngle = landingAngle;
        this.failureReason = failureReason;
    }

    /**
     * Creates a successful solution.
     */
    public static LaunchSolution success(double launchVelocity, double pitchAngle, double yawAngle,
                                         double flightTime, Translation3d launchVelocityVector,
                                         Translation3d predictedLandingPosition, double landingAngle) {
        return new LaunchSolution(true, launchVelocity, pitchAngle, yawAngle, flightTime,
                                  launchVelocityVector, predictedLandingPosition, landingAngle, null);
    }

    /**
     * Creates a failed solution with a reason.
     */
    public static LaunchSolution failure(String reason) {
        return new LaunchSolution(false, 0, 0, 0, 0, Translation3d.ZERO, Translation3d.ZERO, 0, reason);
    }

    public boolean isSolutionFound() {
        return solutionFound;
    }

    /**
     * @return Launch speed in m/s (relative to launcher, not ground)
     */
    public double getLaunchVelocity() {
        return launchVelocity;
    }

    /**
     * @return Pitch angle in radians (elevation)
     */
    public double getPitchAngle() {
        return pitchAngle;
    }

    /**
     * @return Pitch angle in degrees (elevation)
     */
    public double getPitchAngleDegrees() {
        return Math.toDegrees(pitchAngle);
    }

    /**
     * @return Yaw angle in radians (horizontal direction, world frame)
     */
    public double getYawAngle() {
        return yawAngle;
    }

    /**
     * @return Yaw angle in degrees (horizontal direction, world frame)
     */
    public double getYawAngleDegrees() {
        return Math.toDegrees(yawAngle);
    }

    /**
     * @return Time of flight in seconds
     */
    public double getFlightTime() {
        return flightTime;
    }

    /**
     * @return The complete launch velocity vector in world coordinates
     */
    public Translation3d getLaunchVelocityVector() {
        return launchVelocityVector;
    }

    /**
     * @return Predicted landing position
     */
    public Translation3d getPredictedLandingPosition() {
        return predictedLandingPosition;
    }

    /**
     * @return Landing angle in radians (angle of descent, positive = coming down)
     */
    public double getLandingAngle() {
        return landingAngle;
    }

    /**
     * @return Landing angle in degrees
     */
    public double getLandingAngleDegrees() {
        return Math.toDegrees(landingAngle);
    }

    /**
     * @return Reason for failure if no solution found
     */
    public String getFailureReason() {
        return failureReason;
    }

    @Override
    public String toString() {
        if (!solutionFound) {
            return "LaunchSolution[FAILED: " + failureReason + "]";
        }
        return String.format(
            "LaunchSolution[velocity=%.2f m/s, pitch=%.2f°, yaw=%.2f°, flightTime=%.3fs, landingAngle=%.2f°]",
            launchVelocity,
            Math.toDegrees(pitchAngle),
            Math.toDegrees(yawAngle),
            flightTime,
            Math.toDegrees(landingAngle)
        );
    }
}
