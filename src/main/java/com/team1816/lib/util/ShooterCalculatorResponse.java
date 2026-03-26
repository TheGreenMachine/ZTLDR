package com.team1816.lib.util;

public class ShooterCalculatorResponse {
    private double inclineAngleDegrees = 0;
    private double launchVelocityRPS = 0;

    ShooterCalculatorResponse(double inclineAngleDegrees, double launchVelocityRPS) {
        this.inclineAngleDegrees = inclineAngleDegrees;
        this.launchVelocityRPS = launchVelocityRPS;
    }

    public double getInclineAngelDegrees() {
        return inclineAngleDegrees;
    }

    public double getLaunchVelocityRPS() {
        return launchVelocityRPS;
    }
}
