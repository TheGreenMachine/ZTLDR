package com.team1816.lib;

import com.team1816.lib.util.FieldContainer;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.wpilibj.TimedRobot;

public abstract class BaseRobot extends TimedRobot {
    private final BaseRobotContainer baseRobotContainer;
    private long lastVisionErrorLogMs = 0;

    protected BaseRobot() {
        baseRobotContainer = createRobotContainer();

        // The loop time in seconds for adding the vision measurements to the drivetrain pose
        // estimate. For comparison, the main robot periodic loop time is 0.02 seconds (20
        // milliseconds).
        final double addVisionMeasurementsLoopTimeSeconds = 0.02;
        // Add a periodic method to add the vision measurements to the drivetrain pose estimate
        // faster than the main robot loop to make sure we always have the most up-to-date pose
        // estimate.
        addPeriodic(
            () -> {
                try {
                    baseRobotContainer.addVisionMeasurementsToDrivetrain();
                }
                catch (Throwable t) {
                    // Rate-limit error logging to once per second to avoid OOM from
                    // allocating stack trace strings at 250Hz
                    long now = System.currentTimeMillis();
                    if (now - lastVisionErrorLogMs > 1000) {
                        GreenLogger.log(t);
                        lastVisionErrorLogMs = now;
                    }
                }
            },
            addVisionMeasurementsLoopTimeSeconds
        );
    }

    @Override
    public void robotPeriodic() {
        // Update the pose of the robot on the field.
        FieldContainer.field.setRobotPose(BaseRobotState.robotPose);
        // Update the path preview until the match starts.
        if (!BaseRobotState.hasAutoStarted) {
            baseRobotContainer.autoModeManager.updateAutoPathPreviewDisplay();
        }
    }

    @Override
    public void autonomousInit() {
        BaseRobotState.hasAutoStarted = true;
        // Clear the path preview at the start of the match.
        baseRobotContainer.autoModeManager.clearAutoPathPreviewDisplay();
    }

    /**
     * This method should be overridden in the season specific implementation of {@link BaseRobot}
     * to create the instance of the season specific implementation of {@link BaseRobotContainer}.
     * It should then return this instance to allow {@link BaseRobot} to access it.
     * <p>
     * It is important that the overriding method is where the new instance of the {@link
     * BaseRobotContainer} implementation is actually created, instead of initializing it somewhere
     * else like at the top of the class. This is because this method will be called in the
     * constructor of {@link BaseRobot}, meaning that it will run before <i>any</i> code in the
     * season {@link BaseRobot} implementation is run, causing all fields initialized elsewhere to
     * still be null at this point.
     *
     * @return The instance of the season specific implementation of {@link BaseRobotContainer}
     * from the season specific implementation of {@link BaseRobot}.
     */
    protected abstract BaseRobotContainer createRobotContainer();
}
