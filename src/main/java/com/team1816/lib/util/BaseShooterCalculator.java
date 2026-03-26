package com.team1816.lib.util;

import com.team1816.lib.BaseRobotState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class BaseShooterCalculator implements IShooterCalculator {
    public Rotation2d getTurrentAngle(Translation2d shooter, Translation2d target) {
        Translation2d shooterToTargetTranslation2d = target.minus(shooter);
        Rotation2d fieldRelativeRotation2dToTarget = shooterToTargetTranslation2d.getAngle();
        Rotation2d robotRotation2d = BaseRobotState.robotPose.getRotation();
        return fieldRelativeRotation2dToTarget.minus(robotRotation2d);
    }

}
