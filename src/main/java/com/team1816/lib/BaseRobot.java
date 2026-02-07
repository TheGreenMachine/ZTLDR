package com.team1816.lib;

import edu.wpi.first.wpilibj.TimedRobot;

public abstract class BaseRobot<T extends BaseRobotContainer> extends TimedRobot {
    protected final T robotContainer;

    /**
     * The loop time in seconds for adding the vision measurements to the drivetrain pose estimate.
     * For comparison, the main robot periodic loop time is 0.02 seconds (20 milliseconds).
     */
    private final double addVisionMeasurementsLoopTimeSeconds = 0.005;

    protected BaseRobot(T robotContainer) {
        this.robotContainer = robotContainer;

        // Add a periodic method to add the vision measurements to the drivetrain pose estimate
        // faster than the main robot loop to make sure we always have the most up-to-date pose
        // estimate.
        addPeriodic(
            robotContainer::addVisionMeasurementsToDrivetrain,
            addVisionMeasurementsLoopTimeSeconds
        );
    }
}
