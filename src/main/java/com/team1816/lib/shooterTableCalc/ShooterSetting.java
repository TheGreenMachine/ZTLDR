package com.team1816.lib.shooterTableCalc;

public class ShooterSetting {
    private double elevation;
    private double power;

    public ShooterSetting (double elevation, double power) {
        this.elevation = elevation;
        this.power = power;
    }

    public double getElevation() { return elevation; }
    public double getPower() { return power; }
}
