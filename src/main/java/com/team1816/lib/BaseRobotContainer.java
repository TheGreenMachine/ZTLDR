package com.team1816.lib;

import com.team1816.lib.auto.AutoModeManager;
import com.team1816.lib.auto.PathfindManager;
import com.team1816.lib.inputs.CommandButtonBoard;
import com.team1816.lib.subsystems.BaseSuperstructure;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.Vision;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import com.team1816.lib.subsystems.vision.VisionConfiguration;
import com.team1816.lib.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public abstract class BaseRobotContainer {
    private final BaseSuperstructure baseSuperstructure;
    protected CommandXboxController driverController = new CommandXboxController(0);
    protected CommandXboxController operatorController = new CommandXboxController(1);
    protected CommandButtonBoard buttonBoard = new CommandButtonBoard(2);

    protected final Swerve swerve = new Swerve(driverController);
    public final VisionSubsystem visionSubsystem = new VisionSubsystem(
        BaseConstants.VisionConstants.CameraConstants.cameras,
        BaseConstants.VisionConstants.aprilTagFieldLayout,
        new VisionConfiguration(),
        swerve::addVisionMeasurement,
        swerve
    );

    protected final Vision vision = new Vision();

    protected PathfindManager pathfindManager;
    public AutoModeManager autoModeManager;

    protected BaseRobotContainer() {
        baseSuperstructure = createSuperstructure();
    }

    public void initializeLibSubSystems() {
        Singleton.CreateSubSystem(LedManager.class);

        pathfindManager = Singleton.get(PathfindManager.class);
        autoModeManager = new AutoModeManager(swerve::resetPose);
    }

    /**
     * Tells the superstructure to add vision measurements to the drivetrain pose estimate.
     */
    public void addVisionMeasurementsToDrivetrain() {
        baseSuperstructure.addVisionMeasurementsToDrivetrain();
    }

    /**
     * This method should be overridden in the season specific implementation of {@link
     * BaseRobotContainer} to create the instance of the season specific implementation of {@link
     * BaseSuperstructure}. It should then return this instance to allow {@link BaseRobotContainer}
     * to access it.
     * <p>
     * It is important that the overriding method is where the new instance of the {@link
     * BaseSuperstructure} implementation is actually created, instead of initializing it somewhere
     * else like at the top of the class. This is because this method will be called in the
     * constructor of {@link BaseRobotContainer}, meaning that it will run before <i>any</i> code
     * in the season {@link BaseRobotContainer} implementation is run, causing all fields
     * initialized elsewhere to still be null at this point.
     *
     * @return The instance of the season specific implementation of {@link BaseRobotContainer}
     * from the season specific implementation of {@link BaseRobot}.
     */
    protected abstract BaseSuperstructure createSuperstructure();
}
