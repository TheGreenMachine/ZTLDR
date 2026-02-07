package com.team1816.lib;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PPLibTelemetry;
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

public abstract class BaseRobotContainer<T extends BaseSuperstructure> {
    protected final T superstructure;
    protected CommandXboxController controller = new CommandXboxController(0);

    public SendableChooser<Command> autoChooser;
    public final Swerve swerve = new Swerve(controller);
    public final Vision vision = new Vision();
    private boolean poseInitialized;

    protected BaseRobotContainer() {
        superstructure = createSuperstructure();
    }

    protected abstract T createSuperstructure();

    public void initializeLibSubSystems() {
        Singleton.CreateSubSystem(LedManager.class);
    }

    public void initializeAutonomous() {
        autoChooser = AutoBuilder.buildAutoChooser(Singleton.factory.getDefaultAuto());
        SmartDashboard.putData("Auto Mode", autoChooser);
        autoChooser.onChange(this::updatePoseOnSelection);
    }

    public void updateInitialPose(){
        if(poseInitialized || DriverStation.getAlliance().isEmpty()) return;
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
        superstructure.addVisionMeasurementsToDrivetrain();
    }
}
