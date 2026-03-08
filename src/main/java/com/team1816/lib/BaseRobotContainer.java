package com.team1816.lib;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.team1816.lib.auto.PathfindManager;
import com.team1816.lib.inputs.ButtonBoard;
import com.team1816.lib.inputs.CommandButtonBoard;
import com.team1816.lib.subsystems.BaseSuperstructure;
import com.team1816.lib.subsystems.LedManager;
import com.team1816.lib.subsystems.Vision;
import com.team1816.lib.subsystems.drivetrain.Swerve;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public abstract class BaseRobotContainer {
    private final BaseSuperstructure baseSuperstructure;
    protected CommandXboxController driverController = new CommandXboxController(0);
    protected CommandXboxController operatorController = new CommandXboxController(1);
    protected CommandButtonBoard buttonBoard = new CommandButtonBoard(2);

    public SendableChooser<Command> autoChooser;
    protected final Swerve swerve = new Swerve(driverController);
    protected final Vision vision = new Vision();
    private boolean poseInitialized;

    protected PathfindManager pathfindManager;

    protected BaseRobotContainer() {
        baseSuperstructure = createSuperstructure();
    }

    public void initializeLibSubSystems() {
        Singleton.CreateSubSystem(LedManager.class);

        pathfindManager = Singleton.get(PathfindManager.class);
    }

    public void buildAutoChooser() {
        autoChooser = AutoBuilder.buildAutoChooser(Singleton.factory.getDefaultAuto());
        SmartDashboard.putData("Auto Mode", autoChooser);
        autoChooser.onChange(this::updatePoseOnSelection);
    }

    public void updateInitialPose(){
        if(poseInitialized || DriverStation.getAlliance().isEmpty()) return;
        forceUpdatePose();
    }

    /**
     * Forces pose update regardless of poseInitialized state.
     * Called from autonomousInit to ensure pose is always set before auto starts.
     */
    public void forceUpdatePose(){
        updatePoseOnSelection(autoChooser.getSelected());
    }

    private void updatePoseOnSelection(Command selectedAuto) {
        if (selectedAuto != null) {
            try {
                // Load the PathPlanner auto
                PathPlannerAuto auto = (PathPlannerAuto) selectedAuto;
                // Get the starting pose of the first path in the auto
                Pose2d startingPose = auto.getStartingPose();
                if (startingPose != null) {
                    var alliance = DriverStation.getAlliance();
                    if (!alliance.isEmpty() && alliance.get() == DriverStation.Alliance.Red) {
                        startingPose = FlippingUtil.flipFieldPose(startingPose);
                    }
                    // Reset odometry and update Field2d this is to give drivers clue that the
                    // proper auto is set prior to auto start
                    GreenLogger.log("Init " + startingPose);
                    swerve.resetPose(startingPose);
                    PPLibTelemetry.setTargetPose(startingPose);
                    poseInitialized = true;
                }
            } catch (Exception e) {
                GreenLogger.log("Error loading auto pose: " + e.getMessage());
            }
        }
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
