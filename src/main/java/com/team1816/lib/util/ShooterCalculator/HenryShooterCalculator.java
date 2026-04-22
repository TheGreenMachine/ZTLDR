package com.team1816.lib.util.ShooterCalculator;

import com.team1816.lib.BaseRobotState;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class HenryShooterCalculator implements IShooterCalculator {

    private final LinearMPSToLauncherRPSLookup linearMPSToLauncherRPSLookup;

    /**
     * Construct a {@link HenryShooterCalculator} to calculate how to shoot a projectile into a
     * target at a given angle of entry.
     *
     * @param deltaZMeters The difference between the target height and shooter height used when
     *                     calibrating the shooterSettings so we can use those settings to
     *                     determine the relationship between launcher RPS and linear exit
     *                     velocity, in meters.
     */
    public HenryShooterCalculator(double deltaZMeters) {
        linearMPSToLauncherRPSLookup = new LinearMPSToLauncherRPSLookup(deltaZMeters);
    }

    /**
     * Calculate the necessary settings to shoot from the shooter to the target. This calculator
     * uses physics to determine the mechanism settings to enter the target as close to the passed
     * in angle of entry as possible. It also uses a lookup table to convert from the calculated
     * linear launch velocity to the necessary launch motor RPS.
     *
     * @param shooterTranslation3dMeters The field-relative {@link Translation3d} of where the
     *                                   shooter is shooting from, in meters.
     * @param targetTranslation3dMeters The field-relative {@link Translation3d} of where to shoot
     *                                  to, in meters.
     * @param angleOfEntryDegrees The desired angle of entry of the projectile into the target, in
     *                            degrees up from horizontal, in meters.
     * @param useChassisSpeedForHoodAngleAndSpeed Unused for this calculator.
     * @param lookAheadTimeSeconds The time to project forward the shooter's position before
     *                             performing calculations, in seconds. This can be used to give
     *                             mechanisms that cannot keep up with the robot's movement a head
     *                             start. However, it should be used with caution, because it will
     *                             not behave well while accelerating or turning.
     * @return A {@link com.team1816.lib.util.ShooterCalculator.IShooterCalculator.ShooterCalculatorResponse}
     * describing how to shoot the projectile from the shooter to the target.
     */
    public IShooterCalculator.ShooterCalculatorResponse calculate(
        Translation3d shooterTranslation3dMeters,
        Translation3d targetTranslation3dMeters,
        double angleOfEntryDegrees,
        boolean useChassisSpeedForHoodAngleAndSpeed,
        double lookAheadTimeSeconds
    ) {
        // Acceleration due to gravity near earth's surface in m/s² - change this if we are
        // competing on a different planet
        final double g = 9.80665;

        // Get the field-relative ChassisSpeeds from the robot-relative ChassisSpeeds
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            BaseRobotState.robotSpeeds, BaseRobotState.robotPose.getRotation()
        );
        // Current field-relative chassis (robot) velocity components in m/s
        double vCX = chassisSpeeds.vxMetersPerSecond;
        double vCY = chassisSpeeds.vyMetersPerSecond;

        // Project ahead where the shooter will be after lookAheadTimeSeconds based on the robot's
        // velocity
        Translation3d projectedShooterTranslation3dMeters = shooterTranslation3dMeters.plus(
            new Translation3d(vCX * lookAheadTimeSeconds, vCY * lookAheadTimeSeconds, 0)
        );

        Translation3d deltaTranslation3d = targetTranslation3dMeters.minus(
            projectedShooterTranslation3dMeters
        );
        // Distance from start to target components in meters
        double deltaX = deltaTranslation3d.getX();
        double deltaY = deltaTranslation3d.getY();
        double deltaZ = deltaTranslation3d.getZ();

        // Time of travel through the air in seconds - depends only on positions and angle of entry
        double time = Math.sqrt(
            2
            * (
                deltaZ
                + (
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
