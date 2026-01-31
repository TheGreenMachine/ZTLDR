package com.team1816.lib.util.ballisticCalc.src.main.java.com.ballistic;

/**
 * Defines the physical constraints of the launcher mechanism.
 */
public class LauncherConstraints {
    private final double minLaunchVelocity;  // m/s
    private final double maxLaunchVelocity;  // m/s
    private final double minPitchAngle;      // radians (negative = down)
    private final double maxPitchAngle;      // radians (positive = up)
    private final double minYawAngle;        // radians (relative to vehicle forward)
    private final double maxYawAngle;        // radians (relative to vehicle forward)

    /**
     * Creates launcher constraints.
     *
     * @param minLaunchVelocity Minimum launch speed in m/s
     * @param maxLaunchVelocity Maximum launch speed in m/s
     * @param minPitchAngleDegrees Minimum pitch angle in degrees (negative = down)
     * @param maxPitchAngleDegrees Maximum pitch angle in degrees (positive = up)
     * @param minYawAngleDegrees Minimum yaw angle in degrees (relative to forward)
     * @param maxYawAngleDegrees Maximum yaw angle in degrees (relative to forward)
     */
    public LauncherConstraints(
            double minLaunchVelocity,
            double maxLaunchVelocity,
            double minPitchAngleDegrees,
            double maxPitchAngleDegrees,
            double minYawAngleDegrees,
            double maxYawAngleDegrees) {
        this.minLaunchVelocity = minLaunchVelocity;
        this.maxLaunchVelocity = maxLaunchVelocity;
        this.minPitchAngle = Math.toRadians(minPitchAngleDegrees);
        this.maxPitchAngle = Math.toRadians(maxPitchAngleDegrees);
        this.minYawAngle = Math.toRadians(minYawAngleDegrees);
        this.maxYawAngle = Math.toRadians(maxYawAngleDegrees);
    }

    /**
     * Creates default constraints with reasonable values.
     * Velocity: 1-30 m/s, Pitch: -10° to 80°, Yaw: -180° to 180°
     */
    public static LauncherConstraints defaults() {
        return new LauncherConstraints(1.0, 30.0, -10.0, 80.0, -180.0, 180.0);
    }

    public double getMinLaunchVelocity() {
        return minLaunchVelocity;
    }

    public double getMaxLaunchVelocity() {
        return maxLaunchVelocity;
    }

    public double getMinPitchAngle() {
        return minPitchAngle;
    }

    public double getMaxPitchAngle() {
        return maxPitchAngle;
    }

    public double getMinYawAngle() {
        return minYawAngle;
    }

    public double getMaxYawAngle() {
        return maxYawAngle;
    }

    public boolean isVelocityValid(double velocity) {
        return velocity >= minLaunchVelocity && velocity <= maxLaunchVelocity;
    }

    public boolean isPitchValid(double pitchRadians) {
        return pitchRadians >= minPitchAngle && pitchRadians <= maxPitchAngle;
    }

    public boolean isYawValid(double yawRadians) {
        return yawRadians >= minYawAngle && yawRadians <= maxYawAngle;
    }
}
