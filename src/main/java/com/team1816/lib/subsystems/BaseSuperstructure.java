package com.team1816.lib.subsystems;

import com.team1816.lib.subsystems.drivetrain.Swerve;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

/**
 * Base class for season-specific superstructures. Owns the swerve drivetrain and
 * vision subsystem references and forwards fused vision measurements to the
 * drivetrain's pose estimator.
 * <p>
 * The vision pipeline is {@link VisionRedux}: a single fused measurement per cycle
 * (with realistic std devs and gyro-locked rotation) is fed to the Kalman filter.
 * No pose-loss state machine, no dynamic state-stddev manipulation — VisionRedux's
 * output is honest about its uncertainty and the Kalman filter handles the rest.
 */
public abstract class BaseSuperstructure extends SubsystemBase {
    protected final Swerve swerve;
    protected final VisionRedux vision;

    protected BaseSuperstructure(Swerve swerve, VisionRedux vision) {
        this.swerve = swerve;
        this.vision = vision;
    }

    /**
     * Pulls one fused vision measurement from {@link VisionRedux} (if available this
     * cycle) and feeds it to the drivetrain's pose estimator.
     * <p>
     * Called from the vision thread (via {@link com.team1816.lib.BaseRobot}'s
     * {@code addPeriodic}).
     */
    public void addVisionMeasurementsToDrivetrain() {
        Optional<VisionRedux.VisionMeasurement> maybeMeasurement = vision.processFrame();
        if (maybeMeasurement.isEmpty()) return;

        VisionRedux.VisionMeasurement m = maybeMeasurement.get();
        swerve.addVisionMeasurement(m.pose(), m.timestampSeconds(), m.stdDevs());
    }
}
