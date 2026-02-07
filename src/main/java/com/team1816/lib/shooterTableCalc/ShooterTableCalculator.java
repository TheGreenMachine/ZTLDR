package com.team1816.lib.shooterTableCalc;

import com.team1816.season.subsystems.Shooter;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.Map;

public class ShooterTableCalculator {
    private Map<Double, ShooterSetting> shooterTable = Map.of(
        1.2, new ShooterSetting(35, 0.07),
        1.4, new ShooterSetting(40, 0.09)
    );

    public ShooterSetting getShooterSetting(double distance) {
        return shooterTable.get(distance);
    }


}
