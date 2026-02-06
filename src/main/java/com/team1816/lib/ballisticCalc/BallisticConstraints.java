package com.team1816.lib.ballisticCalc;

public class BallisticConstraints {
    /**
     * This will be where the turret and incline restraints will be. DO NOT send the motor to anything, these will all ultimately be getters.
     */
    private double enterAngle;
    public double getEnterAngle (double enterAngle) {
        this.enterAngle = enterAngle;
        return enterAngle;
    }
}
