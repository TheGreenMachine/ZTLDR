package com.team1816.lib.util;

import com.team1816.lib.BaseRobotState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface IShooterCalculator {
    Rotation2d getTurretAngle(Translation2d turret, Translation2d target, boolean useChassisSpeedForHoodAngleAndSpeed);
    ShooterCalculatorResponse getShooterSettings(Pose2d robotPose, ChassisSpeeds groundSpeed, Translation2d target);

    record ShooterCalculatorResponse(double inclineAngleDegrees, double launchVelocityRPS, Rotation2d targetHeading) {}
}
