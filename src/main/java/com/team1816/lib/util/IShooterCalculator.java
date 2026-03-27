package com.team1816.lib.util;

import com.team1816.lib.BaseRobotState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface IShooterCalculator {
    Rotation2d getTurretAngle(Translation2d turret, Translation2d target, boolean useChassisSpeedForHoodAngleAndSpeed);
    ShooterCalculatorResponse getShooterSettings(Translation2d turret, Translation2d target, boolean useChassisSpeedForHoodAngleAndSpeed);

    record ShooterCalculatorResponse(double inclineAngleDegrees, double launchVelocityRPS) {}
}
