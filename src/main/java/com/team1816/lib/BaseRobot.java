package com.team1816.lib;

import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public abstract class BaseRobot extends TimedRobot {
    private final BaseRobotContainer baseRobotContainer;

    /**
     * A Field2d to display the pose of the robot to the drive team on the Elastic dashboard.
     */
    private final Field2d field = new Field2d();

    protected BaseRobot() {
        baseRobotContainer = createRobotContainer();

        GreenLogger.periodicLog("Field", () -> field);

        // The loop time in seconds for adding the vision measurements to the drivetrain pose
        // estimate. For comparison, the main robot periodic loop time is 0.02 seconds (20
        // milliseconds).
        final double addVisionMeasurementsLoopTimeSeconds = 0.005;
        // Add a periodic method to add the vision measurements to the drivetrain pose estimate
        // faster than the main robot loop to make sure we always have the most up-to-date pose
        // estimate.
        addPeriodic(
            baseRobotContainer::addVisionMeasurementsToDrivetrain,
            addVisionMeasurementsLoopTimeSeconds
        );
    }

    @Override
    public void robotPeriodic() {
        // Update the pose of the robot on the field.
        field.setRobotPose(BaseRobotState.robotPose);
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
