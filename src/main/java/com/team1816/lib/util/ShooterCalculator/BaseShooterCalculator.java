package com.team1816.lib.util.ShooterCalculator;

import com.team1816.lib.BaseRobotState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public abstract class BaseShooterCalculator implements IShooterCalculator {
    protected Rotation2d getTurretAngle(Translation2d shooter, Translation2d target, boolean useChassisSpeedForHoodAngleAndSpeed) {
        Translation2d shooterToTargetTranslation2d = target.minus(shooter);
        Rotation2d fieldRelativeRotation2dToTarget = shooterToTargetTranslation2d.getAngle();
        Rotation2d robotRotation2d = BaseRobotState.robotPose.getRotation();
        // Get the robot relative rotation 2d.
        return fieldRelativeRotation2dToTarget.minus(robotRotation2d);
    }
}
