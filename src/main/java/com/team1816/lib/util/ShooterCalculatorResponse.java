package com.team1816.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterCalculatorResponse {
    private double inclineAngleDegrees = 0;
    private double launchVelocityRPS = 0;
    private Rotation2d turretAngle = null;

    ShooterCalculatorResponse(Rotation2d turretAngle, double inclineAngleDegrees, double launchVelocityRPS) {
        this.turretAngle = turretAngle;
        this.inclineAngleDegrees = inclineAngleDegrees;
        this.launchVelocityRPS = launchVelocityRPS;
    }

    public double getInclineAngelDegrees() {
        return inclineAngleDegrees;
    }

    public double getLaunchVelocityRPS() {
        return launchVelocityRPS;
    }

    public Rotation2d getTurrentAngle() {
        return turretAngle;
    }
}
