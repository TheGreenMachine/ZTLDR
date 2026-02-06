package com.team1816.lib.ballisticCalc;

import edu.wpi.first.math.geometry.Pose2d;

public class BallisticSolution {
    private double rotationAngle;
    private double launchAngle;
    private double launchVelocity;

    public BallisticSolution(double rotationAngle, double launchAngle, double launchVelocity) {
        this.rotationAngle = rotationAngle;
        this.launchAngle = launchAngle;
        this.launchVelocity = launchVelocity;
    }

    public double getRotationAngle() { return rotationAngle; }

    public double getLaunchAngle() { return launchAngle; }

    public double getLaunchVelocity() { return launchVelocity; }
}
