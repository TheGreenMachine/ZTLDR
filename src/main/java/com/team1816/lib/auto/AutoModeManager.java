package com.team1816.lib.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.team1816.lib.util.GreenLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Optional;
import java.util.function.Consumer;

/**
 * Manages SmartDashboard and logging of autos. Does NOT interact directly with {@link CommandScheduler} or the drivetrain.
 */
public class AutoModeManager {
    /**
     * Puts dropdown in SmartDashboard and adds all autos.
     */
    public AutoModeManager() {
        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Mode", autoChooser);
        autoChooser.setDefaultOption("None", null);

        listener = a -> {};

        autoChooser.onChange(a -> {
            // log starting pose
            Pose2d startingPose = a.getStartingPose();
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                startingPose = FlippingUtil.flipFieldPose(startingPose);
            }
            GreenLogger.log("Auto Mode Manager- starting pose: " + startingPose);
            // TODO: why is this not PPLibTelemetry.setCurrentPose? look into this after Pittsburgh
            PPLibTelemetry.setTargetPose(startingPose);

            if (listener != null) listener.accept(a);
        });

        for (String autoName : AutoBuilder.getAllAutoNames()) {
            autoChooser.addOption(autoName, new PathPlannerAuto(autoName, false));
            autoChooser.addOption("(MIRROR) " + autoName, new PathPlannerAuto(autoName, true));
        }
    }

    private final SendableChooser<PathPlannerAuto> autoChooser;
    private Consumer<PathPlannerAuto> listener;

    /**
     * Retrieves currently selected auto. May be null if "None" is selected.
     */
    public PathPlannerAuto getSelectedAuto() {
        return autoChooser.getSelected();
    }

    /**
     * Registers a single listener to changes to the auto choosers.
     */
    public void onChange(Consumer<PathPlannerAuto> listener) {
        this.listener = listener;
    }
}
