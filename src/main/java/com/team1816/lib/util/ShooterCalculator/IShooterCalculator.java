package com.team1816.lib.util.ShooterCalculator;

import edu.wpi.first.math.geometry.Translation3d;

public interface IShooterCalculator {

    /**
     * Calculate the necessary settings to shoot from the shooter to the target.
     *
     * @param shooterTranslation3dMeters The {@link Translation3d} of where the shooter is shooting
     *                                   from.
     * @param targetTranslation3dMeters The {@link Translation3d} of where to shoot to.
     * @param angleOfEntryDegrees The desired angle of entry of the projectile into the target.
     *                            Not used by all calculator types.
     * @param useChassisSpeedForHoodAngleAndSpeed If the chassis speeds should be factored into
     *                                            calculations for incline angle and launch
     *                                            velocity. Not used by all calculator types.
     * @param lookAheadTimeSeconds The time to project forward the shooter's position before
     *                             performing calculations, in seconds. This can be used to give
     *                             mechanisms that cannot keep up with the robot's movement a head
     *                             start. However, it should be used with caution, because it will
     *                             not behave well while accelerating or turning. Not used by all
     *                             calculator types.
     * @return The {@link ShooterCalculatorResponse} describing how the projectile should be shot
     * to reach the target.
     */
    ShooterCalculatorResponse calculate(
        Translation3d shooterTranslation3dMeters,
        Translation3d targetTranslation3dMeters,
        double angleOfEntryDegrees,
        boolean useChassisSpeedForHoodAngleAndSpeed,
        double lookAheadTimeSeconds
    );

    /**
     * A response returned by a shooter calculator describing how to shoot the projectile.
     *
     * @param turretAngleDegrees The robot-relative turret angle to point at in degrees,
     *                           counterclockwise from robot forward.
     * @param inclineAngleDegrees The incline angle at which the projectile should be launched in
     *                            degrees, down from straight up.
     * @param launchVelocityRPS The velocity to spin the launch motors at in rotations per second.
     */
    record ShooterCalculatorResponse(double turretAngleDegrees, double inclineAngleDegrees, double launchVelocityRPS) {}
}
