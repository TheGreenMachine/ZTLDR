package com.team1816.lib.util.ShooterCalculator;

import com.team1816.lib.BaseRobotState;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class HenryShooterCalculator implements IShooterCalculator{

    private final LinearMPSToLauncherRPSLookup linearMPSToLauncherRPSLookup = new LinearMPSToLauncherRPSLookup();

    /**
     * Calculate the necessary settings to shoot from the shooter to the target. This calculator
     * uses physics to determine the mechanism settings to enter the target as close to the passed
     * in angle of entry as possible. It also uses a lookup table to convert from the calculated
     * linear launch velocity to the necessary launch motor RPS.
     *
     * @param shooterTranslation3dMeters The {@link Translation3d} of where the shooter is shooting
     *                                   from.
     * @param targetTranslation3dMeters The {@link Translation3d} of where to shoot to.
     * @param angleOfEntryDegrees The desired angle of entry of the projectile into the target.
     * @param useChassisSpeedForHoodAngleAndSpeed Unused for this calculator.
     * @return A {@link com.team1816.lib.util.ShooterCalculator.IShooterCalculator.ShooterCalculatorResponse}
     * describing how to shoot the projectile from the shooter to the target.
     */
    public IShooterCalculator.ShooterCalculatorResponse calculate(
        Translation3d shooterTranslation3dMeters,
        Translation3d targetTranslation3dMeters,
        double angleOfEntryDegrees,
        boolean useChassisSpeedForHoodAngleAndSpeed
    ) {
        // Acceleration due to gravity near earth's surface in m/s² - change this if we are
        // competing on a different planet
        final double g = 9.80665;

        Translation3d deltaTranslation3d = targetTranslation3dMeters.minus(shooterTranslation3dMeters);
        // Distance from start to target components in meters
        double deltaX = deltaTranslation3d.getX();
        double deltaY = deltaTranslation3d.getY();
        double deltaZ = deltaTranslation3d.getZ();

        // Time of travel through the air in seconds - depends only on positions and angle of entry
        double time = Math.sqrt(
            2
            * (
                deltaZ
                - (
                    Math.tan(
                        Units.degreesToRadians(angleOfEntryDegrees)
                    )
                    * Math.sqrt(
                        deltaX * deltaX + deltaY * deltaY
                    )
                )
            )
            / g
        );

        // Get the field-relative ChassisSpeeds from the robot-relative ChassisSpeeds
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            BaseRobotState.robotSpeeds, BaseRobotState.robotPose.getRotation()
        );
        // Current field-relative chassis (robot) velocity components in m/s
        double vCX = chassisSpeeds.vxMetersPerSecond;
        double vCY = chassisSpeeds.vyMetersPerSecond;

        // Calculated shooter velocity components in m/s
        double vSX = deltaX / time - vCX;
        double vSY = deltaY / time - vCY;
        double vSZ = deltaZ / time + (0.5 * g * time);

        // Convert from velocity components to the desired values
        double linearLaunchVelocityMPS = Math.sqrt(vSX * vSX + vSY * vSY + vSZ * vSZ);
        double fieldRelativeTurretAngleRadians = Math.atan2(vSY, vSX);
        double inclineAngleRadians = Math.acos(vSZ / linearLaunchVelocityMPS);

        // Calculated m/s to motor rps
        double launchMotorVelocityRPS = linearMPSToLauncherRPSLookup.getLaunchVelocityRPS(
            linearLaunchVelocityMPS
        );
        // Field relative turret angle to robot relative turret angle
        double robotRelativeTurretAngleRadians = fieldRelativeTurretAngleRadians
            - BaseRobotState.robotPose.getRotation().getRadians();
        // Turret angle radians to degrees
        double robotRelativeTurretAngleDegrees = Units.radiansToDegrees(
            robotRelativeTurretAngleRadians
        );
        // Incline angle radians to degrees
        double inclineAngleDegrees = Units.radiansToDegrees(inclineAngleRadians);

        return new IShooterCalculator.ShooterCalculatorResponse(
            robotRelativeTurretAngleDegrees, inclineAngleDegrees, launchMotorVelocityRPS
        );
    }
}
