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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.stream.Stream;

/**
 * Manages SmartDashboard and logging of autos. Does NOT interact directly with {@link CommandScheduler} or the drivetrain.
 */
public class AutoModeManager {
    private final SendableChooser<Command> autoChooser;
    private Consumer<Command> listener;

    /**
     * Puts dropdown in SmartDashboard and adds all autos.
     */
    public AutoModeManager() {
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            autos -> autos.flatMap(auto -> {
                // Get the name of the original auto for PathPlanner to look for.
                String autoName = auto.getName();
                // Don't create a mirrored version if marked with "[DM]" (Don't Mirror).
                if (autoName.startsWith("[DM]")) {
                    // Remove the "[DM]" marker for the display name, and any spaces.
                    auto.setName(autoName.substring(5).stripLeading());
                    return Stream.of(auto);
                }
                // Rename the original auto to specify it is the left version.
                auto.setName("Left " + autoName);
                // Create a mirrored version of the auto.
                PathPlannerAuto mirroredAuto = new PathPlannerAuto(autoName, true);
                // Rename the mirrored auto to specify it is the right version.
                mirroredAuto.setName("Right " + autoName);
                // Return both the original and the mirrored auto.
                return Stream.of(auto, mirroredAuto);
            })
        );
        SmartDashboard.putData("Auto Mode", autoChooser);

        listener = a -> {};

        autoChooser.onChange(a -> {
            // log starting pose
            Pose2d startingPose = a instanceof PathPlannerAuto
                ? ((PathPlannerAuto) a).getStartingPose()
                : Pose2d.kZero;
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                startingPose = FlippingUtil.flipFieldPose(startingPose);
            }
            GreenLogger.log("Auto Mode Manager- starting pose: " + startingPose);
            // TODO: why is this not PPLibTelemetry.setCurrentPose? look into this after Pittsburgh
            PPLibTelemetry.setTargetPose(startingPose);

            if (listener != null) listener.accept(a);
        });
    }

    /**
     * Retrieves currently selected auto.
     */
    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }

    /**
     * Registers a single listener to changes to the auto choosers.
     */
    public void onChange(Consumer<Command> listener) {
        this.listener = listener;
    }
}
