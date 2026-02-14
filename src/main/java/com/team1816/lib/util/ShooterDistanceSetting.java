package com.team1816.lib.util;

import com.team1816.season.subsystems.Shooter;

public class ShooterDistanceSetting {
    private double angle;
    private double power;

    public ShooterDistanceSetting (double angle, double power) {
        this.angle = angle;
        this.power = power;
    }

    public double getAngle() {
        return angle;
    }

    public double getPower() {
        return power;
    }
}
