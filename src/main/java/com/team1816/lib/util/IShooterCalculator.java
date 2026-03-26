package com.team1816.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

public interface IShooterCalculator {
    ShooterCalculatorResponse getShooterSettings(Translation2d turret, Translation2d target, boolean useChassisSpeedForHoodAngleAndSpeed);
}
